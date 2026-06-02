"""
turretDemo.py — pan and tilt sweep demonstration
=================================================
Designed for video recording of the pan/tilt mechanism.

  Phase 1: pan sweeps from -200 → +200 → -200 → ... × 5 passes, return to 0
  Phase 2: tilt sweeps from +100 → -100 → +100 → ... × 5 passes, return to 0

HOW TO RUN
----------
    cp JuliaFiles/turretDemo.py main.py
    ros2 run robot robot

  BTN_1 — start the demo
  BTN_2 — cancel at any time and return to IDLE
"""

from __future__ import annotations

import time

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
# Hardware
# ---------------------------------------------------------------------------

PAN_STEPPER  = Stepper.STEPPER_1
TILT_STEPPER = Stepper.STEPPER_2

PAN_MAX_VEL  = 600   # steps/s — smooth sweep speed
PAN_ACCEL    = 300   # steps/s²

TILT_MAX_VEL = 250   # steps/s
TILT_ACCEL   = 150   # steps/s²

MOVE_TIMEOUT_S = 10.0   # max seconds per individual move

# ---------------------------------------------------------------------------
# Demo sequence
# ---------------------------------------------------------------------------

# Pan: start at 0, sweep to -200 (setup), then 5 passes across full range, return to 0
PAN_POSITIONS = [-200, +200, -200, +200, -200, +200, -200, +200, -200, +200, 0]

# Tilt: start at 0, sweep to +100 (setup), then 5 passes across full range, return to 0
TILT_POSITIONS = [+100, -100, +100, -100, +100, -100, +100, -100, +100, -100, 0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.BLUE, 0)


def show_pan_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)
    robot.set_led(LED.BLUE, 0)


def show_tilt_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.BLUE, 200)


# ---------------------------------------------------------------------------
# run()
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state       = "INIT"
    task_handle = None

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
            show_idle_leds(robot)
            print("[DEMO] Ready — press BTN_1 to start, BTN_2 to cancel")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):

                def _demo_worker(task: TaskHandle) -> None:
                    # ── Phase 1: pan sweeps ───────────────────────────────
                    show_pan_leds(robot)
                    print(f"[DEMO] Phase 1: pan sweep ×5")
                    for pos in PAN_POSITIONS:
                        if task.cancelled():
                            return
                        ok = robot.step_move(
                            PAN_STEPPER,
                            steps=pos,
                            move_type=StepMoveType.ABSOLUTE,
                            blocking=True,
                            timeout=MOVE_TIMEOUT_S,
                        )
                        if not ok:
                            print(f"[warn] pan move to {pos} timed out")

                    # ── Phase 2: tilt sweeps ──────────────────────────────
                    if task.cancelled():
                        return
                    show_tilt_leds(robot)
                    print(f"[DEMO] Phase 2: tilt sweep ×5")
                    for pos in TILT_POSITIONS:
                        if task.cancelled():
                            return
                        ok = robot.step_move(
                            TILT_STEPPER,
                            steps=pos,
                            move_type=StepMoveType.ABSOLUTE,
                            blocking=True,
                            timeout=MOVE_TIMEOUT_S,
                        )
                        if not ok:
                            print(f"[warn] tilt move to {pos} timed out")

                    print("[DEMO] Complete")

                task_handle = run_task(_demo_worker, blocking=False)
                state = "RUNNING"

        elif state == "RUNNING":
            if robot.was_button_pressed(Button.BTN_2):
                task_handle.cancel()
                task_handle.wait(timeout=MOVE_TIMEOUT_S + 1.0)
                task_handle = None
                show_idle_leds(robot)
                print("[DEMO] Cancelled — press BTN_1 to restart")
                state = "IDLE"
            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                show_idle_leds(robot)
                print("[DEMO] Done — press BTN_1 to run again")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
