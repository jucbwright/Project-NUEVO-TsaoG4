"""
stepper_drive_test.py — Combined loop & single run test for Steppers 2 & 3
===========================================================================
Configured for the NUEVO Board using the robot API.

WHAT IT DOES
------------
  BTN_1 — Single run: both steppers CW 200 steps once
  BTN_2 — Loop test:  both steppers alternate CW/CCW for 10 cycles

HOW TO RUN
----------
    cp stepper_drive_test.py main.py
    ros2 run robot robot
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Stepper configuration — Steppers 2 & 3 (rover drive motors)
# ---------------------------------------------------------------------------
DRIVE_STEPPER_L = Stepper.STEPPER_2    # Left drive motor
DRIVE_STEPPER_R = Stepper.STEPPER_3    # Right drive motor

STEPS_PER_RUN   = 200                  # Steps per move
MAX_VELOCITY    = 800                  # Steps/sec
ACCELERATION    = 400                  # Steps/sec²
STEP_CYCLES     = 10                   # Number of loop cycles
PAUSE_BETWEEN_S = 1.0                  # Pause between cycles (seconds)
MOVE_TIMEOUT_S  = 10.0                 # Timeout per move (seconds)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
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


def enable_drive_steppers(robot: Robot) -> None:
    robot.step_set_config(DRIVE_STEPPER_L, max_velocity=MAX_VELOCITY, acceleration=ACCELERATION)
    robot.step_set_config(DRIVE_STEPPER_R, max_velocity=MAX_VELOCITY, acceleration=ACCELERATION)
    robot.step_enable(DRIVE_STEPPER_L)
    robot.step_enable(DRIVE_STEPPER_R)


def disable_drive_steppers(robot: Robot) -> None:
    robot.step_disable(DRIVE_STEPPER_L)
    robot.step_disable(DRIVE_STEPPER_R)


def move_both(robot: Robot, steps: int) -> bool:
    """Move both steppers the given number of steps. Positive=CW, Negative=CCW."""
    ok_l = robot.step_move(
        DRIVE_STEPPER_L,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=MOVE_TIMEOUT_S,
    )
    ok_r = robot.step_move(
        DRIVE_STEPPER_R,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=MOVE_TIMEOUT_S,
    )
    return ok_l and ok_r


# ---------------------------------------------------------------------------
# Single run — both motors CW once
# ---------------------------------------------------------------------------
def run_single(robot: Robot) -> None:
    print("[SINGLE] Starting — Steppers 2 & 3 CW 200 steps")
    enable_drive_steppers(robot)

    ok = move_both(robot, STEPS_PER_RUN)

    disable_drive_steppers(robot)

    if ok:
        print("[SINGLE] Done")
    else:
        print("[SINGLE] Warning — one or both steppers timed out")


# ---------------------------------------------------------------------------
# Loop test — alternate CW/CCW for STEP_CYCLES cycles
# ---------------------------------------------------------------------------
def run_loop(robot: Robot) -> None:
    print(f"[LOOP] Starting — {STEP_CYCLES} cycles, alternating CW/CCW")
    enable_drive_steppers(robot)

    for ii in range(STEP_CYCLES):
        steps = STEPS_PER_RUN if ii % 2 == 0 else -STEPS_PER_RUN
        direction = "CW" if steps > 0 else "CCW"
        print(f"[LOOP] Cycle {ii + 1}/{STEP_CYCLES} — {direction}")

        ok = move_both(robot, steps)
        if not ok:
            print(f"[LOOP] Warning — timeout on cycle {ii + 1}, stopping early")
            break

        time.sleep(PAUSE_BETWEEN_S)

    disable_drive_steppers(robot)
    print("[LOOP] Done")


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    start_robot(robot)
    show_idle_leds(robot)

    print("[FSM] IDLE — BTN_1 = single run | BTN_2 = loop test")
    print(f"[CFG] Steppers 2 & 3 | steps={STEPS_PER_RUN} vel={MAX_VELOCITY} accel={ACCELERATION}")

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()
    state = "IDLE"

    while True:
        if state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUN_SINGLE")
                state = "RUN_SINGLE"

            elif robot.was_button_pressed(Button.BTN_2):
                show_running_leds(robot)
                print("[FSM] RUN_LOOP")
                state = "RUN_LOOP"

        elif state == "RUN_SINGLE":
            run_single(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — BTN_1 = single run | BTN_2 = loop test")
            state = "IDLE"

        elif state == "RUN_LOOP":
            run_loop(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — BTN_1 = single run | BTN_2 = loop test")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()