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
On startup: both steppers are enabled and current position is treated as (0, 0).
Manually position the mechanism at your desired home before starting.

  BTN_1 -- cycle to the next preset target and aim at it (pan + tilt simultaneously)

Targets are defined in TARGETS as (pan_steps, tilt_steps) absolute positions
from the startup origin.  Add, remove, or adjust entries to match your setup.

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

TILT_MAX_VEL  = 600
TILT_ACCEL    = 300

AIM_TIMEOUT_S  = 10.0   # max seconds to wait for both steppers to reach target
AIM_POLL_S     = 0.02   # polling interval while waiting for moves to finish


# ---------------------------------------------------------------------------
# Pixel -> step conversion  (tune these two constants experimentally)
# ---------------------------------------------------------------------------

IMG_W = 640
IMG_H = 480
HFOV  = 62.2   # degrees, horizontal field of view (Pi Camera 3 default)
VFOV  = 48.8   # degrees, vertical field of view

# Steps per degree of physical rotation -- measure and adjust until accurate.
# To calibrate: command 1000 steps, measure actual degrees rotated,
# then set STEPS_PER_DEG = 1000 / measured_degrees.
PAN_STEPS_PER_DEG  = 10.0
TILT_STEPS_PER_DEG = 10.0


def pixels_to_steps(px: float, py: float) -> tuple[int, int]:
    """Convert pixel coords to absolute pan/tilt steps from origin (center frame = 0,0)."""
    pan_deg  = (px - IMG_W / 2) / IMG_W  * HFOV
    tilt_deg = (py - IMG_H / 2) / IMG_H  * VFOV
    return int(pan_deg * PAN_STEPS_PER_DEG), int(tilt_deg * TILT_STEPS_PER_DEG)


# ---------------------------------------------------------------------------
# Target table -- define in pixel coords; steps are computed automatically.
# Center of frame is (320, 240).  Right/down are positive.
# ---------------------------------------------------------------------------

class Target(NamedTuple):
    label: str
    pan:   int   # absolute steps from startup origin
    tilt:  int   # absolute steps from startup origin

    @staticmethod
    def from_pixels(label: str, px: float, py: float) -> "Target":
        pan, tilt = pixels_to_steps(px, py)
        return Target(label, pan, tilt)


TARGETS: list[Target] = [
    Target.from_pixels("CENTER",     px=320, py=240),   # center of frame
    Target.from_pixels("LEFT-LOW",   px=100, py=380),
    Target.from_pixels("RIGHT-HIGH", px=540, py=100),
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


def setup_steppers(robot: Robot) -> None:
    """Enable both steppers and apply motion config. Current position becomes (0, 0)."""
    robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
    robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
    robot.step_enable(PAN_STEPPER)
    robot.step_enable(TILT_STEPPER)


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
            setup_steppers(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE -- BTN_1 cycles targets (current position is origin)")
            for i, t in enumerate(TARGETS):
                print(f"  [{i}] {t.label}  pan={t.pan}  tilt={t.tilt}")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                state = "AIM"

        elif state == "AIM":
            target = TARGETS[target_index]
            aim_at(robot, target)
            target_index = (target_index + 1) % len(TARGETS)
            show_idle_leds(robot)
            next_target = TARGETS[target_index]
            print(f"[FSM] IDLE -- at {target.label}, BTN_1 -> {next_target.label}")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
