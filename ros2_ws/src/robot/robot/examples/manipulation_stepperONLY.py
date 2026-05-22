"""
manipulation_stepperONLY.py -- aim-and-shoot with two steppers
=============================================================
Controls a pan/tilt shooter mechanism using only stepper motors.

  STEPPER_1 = pan  (side-to-side arc)
  STEPPER_2 = tilt (up/down)

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/manipulation_stepperONLY.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
On startup: both steppers home against their limit switches to establish (0, 0).

  BTN_1 -- cycle to the next preset target and aim at it (pan + tilt simultaneously)
  BTN_3 -- re-home both steppers and reset the target index

Targets are defined in TARGETS as (pan_steps, tilt_steps) absolute positions
from the home origin.  Add, remove, or adjust entries to match your setup.

CAMERA INTEGRATION NOTE
-----------------------
Future: replace the BTN_1 cycle logic with a callback that receives a detected
target's pixel coordinates, converts them to (pan_steps, tilt_steps) via a
calibration matrix, and calls aim_at() directly.  The aim_at() function is
already written for that use case -- it accepts any Target.
"""

from __future__ import annotations

import time
from typing import NamedTuple

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
    StepMoveType,
    Stepper,
    StepperMotionState,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Hardware config -- edit to match your build
# ---------------------------------------------------------------------------

PAN_STEPPER  = Stepper.STEPPER_1
TILT_STEPPER = Stepper.STEPPER_2

PAN_MAX_VEL  = 800    # steps/s
PAN_ACCEL    = 400    # steps/s^2
PAN_HOME_VEL = 300    # steps/s  (slow for reliable homing)

TILT_MAX_VEL  = 600
TILT_ACCEL    = 300
TILT_HOME_VEL = 200

HOME_TIMEOUT_S = 15.0   # max seconds per homing move
AIM_TIMEOUT_S  = 10.0   # max seconds to wait for both steppers to reach target
AIM_POLL_S     = 0.02   # polling interval while waiting for moves to finish


# ---------------------------------------------------------------------------
# Target table -- (pan_steps, tilt_steps) absolute from home origin
# ---------------------------------------------------------------------------

class Target(NamedTuple):
    label: str
    pan:   int   # absolute steps from pan home
    tilt:  int   # absolute steps from tilt home


TARGETS: list[Target] = [
    Target("CENTER",     pan=0,    tilt=0),
    Target("LEFT-LOW",   pan=-500, tilt=300),
    Target("RIGHT-HIGH", pan=500,  tilt=-300),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def _stepper_index(stepper: Stepper) -> int:
    """Convert Stepper enum to zero-based index into StepStateAll.steppers."""
    return int(stepper) - 1


def _both_idle(robot: Robot) -> bool:
    """Return True when both pan and tilt steppers report IDLE motion_state."""
    state = robot.get_step_state()
    if state is None:
        return False
    pan_state  = state.steppers[_stepper_index(PAN_STEPPER)].motion_state
    tilt_state = state.steppers[_stepper_index(TILT_STEPPER)].motion_state
    return (pan_state == StepperMotionState.IDLE and
            tilt_state == StepperMotionState.IDLE)


def _home_one(robot: Robot, stepper: Stepper, home_vel: int, label: str) -> bool:
    print(f"[HOME] homing {label}...")
    robot.step_enable(stepper)
    ok = robot.step_home(
        stepper,
        direction=-1,
        home_velocity=home_vel,
        backoff_steps=50,
        blocking=True,
        timeout=HOME_TIMEOUT_S,
    )
    if not ok:
        print(f"[warn] {label} homing timed out -- check limit switch wiring")
        robot.step_disable(stepper)
        return False
    print(f"[HOME] {label} homed at origin")
    return True


def home_all(robot: Robot) -> bool:
    """Home pan then tilt; both steppers stay enabled after homing."""
    robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
    robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
    if not _home_one(robot, PAN_STEPPER,  PAN_HOME_VEL,  "pan"):
        return False
    if not _home_one(robot, TILT_STEPPER, TILT_HOME_VEL, "tilt"):
        return False
    return True


def aim_at(robot: Robot, target: Target) -> bool:
    """
    Move pan and tilt simultaneously to target's absolute step positions.

    Issues both moves non-blocking so they run in parallel, then polls
    StepperMotionState until both report IDLE (or timeout expires).
    Returns True on success, False on timeout.
    """
    print(f"[AIM] -> {target.label}  pan={target.pan}  tilt={target.tilt}")

    # Start both moves at the same time (non-blocking)
    robot.step_move(PAN_STEPPER,  steps=target.pan,  move_type=StepMoveType.ABSOLUTE, blocking=False)
    robot.step_move(TILT_STEPPER, steps=target.tilt, move_type=StepMoveType.ABSOLUTE, blocking=False)

    # Poll until both are idle or timeout
    deadline = time.monotonic() + AIM_TIMEOUT_S
    while time.monotonic() < deadline:
        if _both_idle(robot):
            print(f"[AIM] aimed at {target.label}")
            return True
        time.sleep(AIM_POLL_S)

    print(f"[warn] aim timed out before reaching {target.label}")
    return False


# ---------------------------------------------------------------------------
# FSM entry point
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    target_index = 0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            show_running_leds(robot)
            ok = home_all(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE -- BTN_1 cycles targets, BTN_3 re-homes")
                for i, t in enumerate(TARGETS):
                    print(f"  [{i}] {t.label}  pan={t.pan}  tilt={t.tilt}")
            else:
                print("[FSM] IDLE -- homing failed; check limit switches before aiming")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                state = "AIM"
            elif robot.was_button_pressed(Button.BTN_3):
                show_running_leds(robot)
                print("[FSM] RE-HOME")
                state = "HOME"

        elif state == "AIM":
            target = TARGETS[target_index]
            aim_at(robot, target)
            target_index = (target_index + 1) % len(TARGETS)
            show_idle_leds(robot)
            next_target = TARGETS[target_index]
            print(f"[FSM] IDLE -- at {target.label}, BTN_1 -> {next_target.label}")
            state = "IDLE"

        elif state == "HOME":
            ok = home_all(robot)
            target_index = 0
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE -- re-homed, target index reset to 0")
            else:
                print("[FSM] IDLE -- re-homing failed")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
