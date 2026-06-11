"""
Transition.py — autonomous handoff from main drive to ZombieSweep
===================================================================
Bridges maindrive_start_stop.py (drive run) and ZombieSweep.py (aim/shoot).

  1. WATCH_STOP    — watches for a stop sign detected in the lower-right
                      quadrant of the camera frame (mirrors ZombieSweep's
                      R_DOWN grid position).
  2. DRIVE_TILE    — once seen, drives forward one tile (TILE_MM, matches
                      maindrive_start_stop.py's grid spacing) using
                      move_forward.
  3. HOME_STEPPERS — re-homes the pan/tilt steppers to (0, 0), the starting
                      position ZombieSweep's INIT assumes.
  4. DONE          — returns; main.py then hands off to ZombieSweep.run().

Usage in main.py
----------------
    from robot.maindrive_start_stop import run as run_drive
    from robot.JuliaFiles.Transition import run as run_transition
    from robot.JuliaFiles.ZombieSweep import run as run_sweep

    def run(robot: Robot) -> None:
        run_drive(robot)
        run_transition(robot)
        run_sweep(robot)
"""

from __future__ import annotations

import time

from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, StepMoveType
from robot.robot import Robot
from robot.JuliaFiles.ZombieSweep import (
    AIM_TIMEOUT_S,
    PAN_ACCEL,
    PAN_MAX_VEL,
    PAN_STEPPER,
    TILT_ACCEL,
    TILT_MAX_VEL,
    TILT_STEPPER,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

MIN_CONFIDENCE   = 0.50   # same threshold as stop.py
VISION_STALE_SEC = 3.0

TILE_MM             = 650.0   # matches maindrive_start_stop.py TILE_MM
DRIVE_VELOCITY_MM_S = 220.0
DRIVE_TOLERANCE_MM  = 25.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.enable_vision()


def _stop_sign_in_lower_right(robot: Robot) -> bool:
    """True if the highest-confidence stop-sign detection this tick is
    centred in the lower-right quadrant of the camera frame."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False

    frame_w, frame_h = robot.get_detection_image_size()
    if frame_w <= 0 or frame_h <= 0:
        return False

    best, best_conf = None, -1.0
    for det in robot.get_detections("stop sign"):
        conf = float(det["confidence"])
        if conf < MIN_CONFIDENCE:
            continue
        if conf > best_conf:
            best_conf = conf
            best = det

    if best is None:
        return False

    bbox = best["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    cy = bbox["y"] + bbox["height"] / 2.0
    return cx > frame_w / 2.0 and cy > frame_h / 2.0


def show_watching_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.RED, 0)


def show_triggered_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.RED, 200)


# ---------------------------------------------------------------------------
# run() - entry point called from main.py between drive and sweep
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "WATCH_STOP"
    drive_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    print("[TRANSITION] WATCH_STOP -- waiting for stop sign in lower-right quadrant")
    show_watching_leds(robot)

    while state != "DONE":

        if state == "WATCH_STOP":
            if _stop_sign_in_lower_right(robot):
                print("[TRANSITION] stop sign seen in lower-right quadrant -- driving forward one tile")
                show_triggered_leds(robot)
                drive_handle = robot.move_forward(
                    TILE_MM,
                    velocity=DRIVE_VELOCITY_MM_S,
                    tolerance=DRIVE_TOLERANCE_MM,
                    blocking=False,
                )
                state = "DRIVE_TILE"

        elif state == "DRIVE_TILE":
            if robot.was_button_pressed(Button.BTN_2):
                drive_handle.cancel()
                drive_handle.wait(timeout=1.0)
                drive_handle = None
                robot.stop()
                print("[TRANSITION] cancelled during DRIVE_TILE")
                state = "DONE"
            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
                robot.stop()
                print("[TRANSITION] tile complete -- re-homing pan/tilt steppers")
                state = "HOME_STEPPERS"

        elif state == "HOME_STEPPERS":
            robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
            robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
            robot.step_enable(PAN_STEPPER)
            robot.step_enable(TILT_STEPPER)
            robot.step_move(PAN_STEPPER,  steps=0, move_type=StepMoveType.ABSOLUTE,
                             blocking=True, timeout=AIM_TIMEOUT_S)
            robot.step_move(TILT_STEPPER, steps=0, move_type=StepMoveType.ABSOLUTE,
                             blocking=True, timeout=AIM_TIMEOUT_S)
            print("[TRANSITION] DONE -- handing off to ZombieSweep")
            state = "DONE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
