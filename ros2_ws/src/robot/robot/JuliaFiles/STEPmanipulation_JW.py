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

Targets are defined in TARGETS as (pan_steps, tilt_steps) absolute positions
from the startup origin.  Add, remove, or adjust entries to match your setup.

CAMERA INTEGRATION NOTE
-----------------------
Future: replace the BTN_1 cycle logic with a callback that receives a detected
target's pixel coordinates, converts them to (pan_steps, tilt_steps) via a
calibration matrix, and calls the sequence directly.
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

TILT_MAX_VEL = 600
TILT_ACCEL   = 300

AIM_TIMEOUT_S = 10.0   # max seconds per stepper move
DWELL_S       = 3.0    # seconds to hold each position before moving to the next


# ---------------------------------------------------------------------------
# Grid positions (steps from startup origin) -- tune to match your build
# ---------------------------------------------------------------------------

PAN_L, PAN_C, PAN_R    = -400,   0,  400   # pan columns: left / center / right
TILT_U, TILT_C, TILT_D =  200,   0, -200   # tilt rows:   up   / center / down


# ---------------------------------------------------------------------------
# Movement sequence -- snake scan across the grid.
# ---------------------------------------------------------------------------

class Target(NamedTuple):
    label: str
    pan:   int    # absolute steps from startup origin (STEPPER_1)
    tilt:  int    # absolute steps from startup origin (STEPPER_2)


SEQUENCE: list[Target] = [
    # Left column — tilt top → bottom
    Target("L_UP",   PAN_L, TILT_U),
    Target("L_CTR",  PAN_L, TILT_C),
    Target("L_DOWN", PAN_L, TILT_D),
    # Center column — tilt bottom → top (snake reversal)
    Target("C_DOWN", PAN_C, TILT_D),
    Target("C_CTR",  PAN_C, TILT_C),
    Target("C_UP",   PAN_C, TILT_U),
    # Right column — tilt top → bottom (snake reversal)
    Target("R_UP",   PAN_R, TILT_U),
    Target("R_CTR",  PAN_R, TILT_C),
    Target("R_DOWN", PAN_R, TILT_D),
    # Return home
    Target("HOME",   PAN_C, TILT_C),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


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

    while True:

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
                        if not task.sleep(DWELL_S):
                            break

                task_handle = run_task(_sequence_worker, blocking=False)
                state = "RUNNING"

        elif state == "RUNNING":
            if robot.was_button_pressed(Button.BTN_2):
                task_handle.cancel()
                task_handle.wait(timeout=AIM_TIMEOUT_S + 1.0)
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE -- cancelled, press BTN_1 to repeat")
                state = "IDLE"
            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE -- sequence complete, press BTN_1 to repeat")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
