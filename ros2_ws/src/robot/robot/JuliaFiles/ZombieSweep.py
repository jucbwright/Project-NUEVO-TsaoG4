"""
STEPmanipulation_JW.py -- aim-and-shoot with two steppers
=============================================================
Controls a pan/tilt shooter mechanism using only stepper motors.

  STEPPER_1 = pan  (side-to-side arc)
  STEPPER_2 = tilt (up/down)

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp JuliaFiles/STEPmanipulation_JW.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
On startup: both steppers are enabled and current position is treated as (0, 0).
Manually position the mechanism at your desired home before starting.

  BTN_1 -- start the aim sequence (pan + tilt through all targets)
  BTN_2 -- cancel the sequence and return to IDLE at any time

At each grid position the turret dwells for DWELL_S seconds and checks the
vision node for zombie detections:
  All LEDs on   = zombie detected at this position
  Orange LED    = no zombie detected

Start the vision node before running:
    ros2 launch vision vision_debug.launch.py
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
)
from robot.robot import FirmwareState, Robot
from robot.util import TaskHandle, run_task


# ---------------------------------------------------------------------------
# Hardware config -- edit to match your build
# ---------------------------------------------------------------------------

PAN_STEPPER  = Stepper.STEPPER_1
TILT_STEPPER = Stepper.STEPPER_2

PAN_MAX_VEL  = 800    # steps/s
PAN_ACCEL    = 400    # steps/s^2

TILT_MAX_VEL = 300
TILT_ACCEL   = 150

AIM_TIMEOUT_S = 10.0   # max seconds per stepper move
DWELL_S       = 3.0    # seconds to hold each position before moving to the next

# ---------------------------------------------------------------------------
# Vision / zombie detection  (same values as zombieTracker.py)
# ---------------------------------------------------------------------------

MIN_CONFIDENCE   = 0.30
VISION_STALE_SEC = 3.0
LED_BRIGHTNESS   = 255
DWELL_POLL_S     = 0.2   # how often to re-check for zombie during the dwell

ALL_LEDS = (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE)


# ---------------------------------------------------------------------------
# Grid positions (steps from startup origin) -- tune to match your build
# ---------------------------------------------------------------------------

PAN_L, PAN_C, PAN_R    = -200,   0,  200   # pan columns: left / center / right
TILT_U, TILT_C, TILT_D =  50,   0, -50  # tilt rows:   up   / center / down


# ---------------------------------------------------------------------------
# Movement sequence -- snake scan across the grid.
# ---------------------------------------------------------------------------

class Target(NamedTuple):
    label: str
    pan:   int    # absolute steps from startup origin (STEPPER_1)
    tilt:  int    # absolute steps from startup origin (STEPPER_2)


SEQUENCE: list[Target] = [
    # Start at centre
    Target("HOME",   PAN_C, TILT_C),
    # Left column — tilt bottom → top
    Target("L_DOWN", PAN_L, TILT_D),
    Target("L_CTR",  PAN_L, TILT_C),
    Target("L_UP",   PAN_L, TILT_U),
    # Centre column — tilt top → bottom (snake reversal)
    Target("C_UP",   PAN_C, TILT_U),
    Target("C_CTR",  PAN_C, TILT_C),
    Target("C_DOWN", PAN_C, TILT_D),
    # Right column — tilt bottom → top (snake reversal)
    Target("R_DOWN", PAN_R, TILT_D),
    Target("R_CTR",  PAN_R, TILT_C),
    Target("R_UP",   PAN_R, TILT_U),
    # Return home
    Target("HOME",   PAN_C, TILT_C),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def _find_zombie(robot: Robot) -> dict | None:
    """Return the highest-confidence zombie detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best, best_conf = None, -1.0
    for det in robot.get_detections("zombie"):
        conf = float(det["confidence"])
        if conf < MIN_CONFIDENCE:
            continue
        if conf > best_conf:
            best_conf = conf
            best = det
    return best


def _dim_all_leds(robot: Robot) -> None:
    for led in ALL_LEDS:
        robot.set_led(led, 0)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# run() - entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state       = "INIT"
    task_handle = None
    pos         = {"pan": 0, "tilt": 0}   # tracks current stepper positions

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while state != "DONE":

        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
            robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
            robot.step_enable(PAN_STEPPER)
            robot.step_enable(TILT_STEPPER)
            pos["pan"] = pos["tilt"] = 0
            show_idle_leds(robot)
            print("[FSM] IDLE -- press BTN_1 to run the aim sequence")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)

                def _sequence_worker(task: TaskHandle) -> None:
                    for target in SEQUENCE:
                        if task.cancelled():
                            break
                        print(f"[AIM] -> {target.label}  pan={target.pan}  tilt={target.tilt}")
                        if target.pan != pos["pan"]:
                            ok = robot.step_move(
                                PAN_STEPPER,
                                steps=target.pan,
                                move_type=StepMoveType.ABSOLUTE,
                                blocking=True,
                                timeout=AIM_TIMEOUT_S,
                            )
                            if not ok:
                                print(f"[warn] pan timed out before reaching {target.label}")
                            pos["pan"] = target.pan
                        if target.tilt != pos["tilt"]:
                            ok = robot.step_move(
                                TILT_STEPPER,
                                steps=target.tilt,
                                move_type=StepMoveType.ABSOLUTE,
                                blocking=True,
                                timeout=AIM_TIMEOUT_S,
                            )
                            if not ok:
                                print(f"[warn] tilt timed out before reaching {target.label}")
                            pos["tilt"] = target.tilt
                        print(f"[AIM] aimed at {target.label} — holding {DWELL_S}s")
                        dwell_end = time.monotonic() + DWELL_S
                        zombie_found = False
                        while time.monotonic() < dwell_end:
                            if task.cancelled():
                                return
                            zombie = _find_zombie(robot)
                            if zombie is not None:
                                zombie_found = True
                                print(f"[ZOMBIE] Detected at {target.label}! Holding 2s for shot...")
                                break
                            _dim_all_leds(robot)
                            robot.set_led(LED.ORANGE, 200)
                            time.sleep(DWELL_POLL_S)

                        if zombie_found:
                            for led in ALL_LEDS:
                                robot.set_led(led, LED_BRIGHTNESS)
                            shoot_end = time.monotonic() + 2.0
                            while time.monotonic() < shoot_end:
                                if task.cancelled():
                                    return
                                time.sleep(DWELL_POLL_S)
                            _dim_all_leds(robot)

                        show_running_leds(robot)

                task_handle = run_task(_sequence_worker, blocking=False)
                state = "RUNNING"

        elif state == "RUNNING":
            if robot.was_button_pressed(Button.BTN_2):
                task_handle.cancel()
                task_handle.wait(timeout=AIM_TIMEOUT_S + 1.0)
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] DONE -- sweep cancelled")
                state = "DONE"
            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] DONE -- sweep complete")
                state = "DONE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
