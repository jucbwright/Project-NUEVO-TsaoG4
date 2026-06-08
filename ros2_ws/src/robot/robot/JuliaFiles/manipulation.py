"""
manipulation.py -- grid scan with pan/tilt + zombie/prisonMike vision detection
================================================================================

Combines STEPmanipulation_JW (stepper grid sequence) with iSeeDeadPpl (zombie
vision detection) to classify what the camera sees at each grid position.

HOW TO RUN
----------
Start the vision node in one terminal (using test images for VM):
    ros2 launch vision vision_debug.launch.py

Copy this file over main.py and restart the robot node:
    cp JuliaFiles/manipulation.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
  BTN_1 -- pan/tilt through every grid position in SEQUENCE.
            At each position the camera scans for DWELL_S seconds:
              - green person detected  → "zombie"    (GREEN LED)
              - non-green person found → "prisonMike" (PURPLE LED)
              - nothing                → no LED
            After the full sequence a summary is printed.
  BTN_2 -- cancel the sequence at any time and return to IDLE.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
    StepMoveType,
)
from robot.robot import FirmwareState, Robot
from robot.util import TaskHandle, run_task

from robot.JuliaFiles.iSeeDeadPpl import (
    dim_all_leds,
    find_zombie,
    LED_BRIGHTNESS,
    MIN_CONFIDENCE,
    VISION_STALE_SEC,
)
from ros2_ws.src.robot.robot.JuliaFiles.ZombieSweep import (
    AIM_TIMEOUT_S,
    DWELL_S,
    PAN_ACCEL,
    PAN_MAX_VEL,
    PAN_STEPPER,
    SEQUENCE,
    show_idle_leds,
    show_running_leds,
    TILT_ACCEL,
    TILT_MAX_VEL,
    TILT_STEPPER,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def find_prison_mike(robot: Robot) -> dict | None:
    """Return the highest-confidence non-zombie person detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best      = None
    best_conf = -1.0

    for detection in robot.get_detections("person"):
        confidence = float(detection["confidence"])
        if confidence < MIN_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color      = attributes.get("color", {}).get("value")

        if color == "green":
            continue  # zombie, not prisonMike

        if confidence > best_conf:
            best_conf = confidence
            best = detection

    return best


# ---------------------------------------------------------------------------
# run() - entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state       = "INIT"
    task_handle = None
    pos         = {"pan": 0, "tilt": 0}  # current stepper position

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
            dim_all_leds(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE -- press BTN_1 to run grid scan")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUNNING -- scanning grid for zombie / prisonMike")

                def _sequence_worker(task: TaskHandle) -> None:
                    scan_results: list[tuple[str, str]] = []

                    for target in SEQUENCE:
                        if task.cancelled():
                            break

                        # ── Aim ──────────────────────────────────────────────
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
                                print(f"[warn] pan timed out before {target.label}")
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
                                print(f"[warn] tilt timed out before {target.label}")
                            pos["tilt"] = target.tilt

                        print(f"[SCAN] at {target.label} — scanning {DWELL_S}s")

                        # ── Scan during dwell ─────────────────────────────────
                        found = "nothing"
                        dwell_end = time.monotonic() + DWELL_S
                        while time.monotonic() < dwell_end:
                            if task.cancelled():
                                scan_results.append((target.label, found))
                                return

                            if find_zombie(robot) is not None:
                                found = "zombie"
                                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                                print(f"[SCAN] {target.label}: ZOMBIE detected!")
                                break
                            elif find_prison_mike(robot) is not None:
                                found = "prisonMike"
                                robot.set_led(LED.PURPLE, LED_BRIGHTNESS)
                                print(f"[SCAN] {target.label}: prisonMike detected!")
                                break

                            time.sleep(0.1)

                        if found == "nothing":
                            print(f"[SCAN] {target.label}: nothing")

                        scan_results.append((target.label, found))

                        # Reset LEDs before moving to next position
                        robot.set_led(LED.GREEN, 0)
                        robot.set_led(LED.PURPLE, 0)

                    # ── Summary ───────────────────────────────────────────────
                    print("\n[SUMMARY] Grid scan results:")
                    for grid_pos, detection in scan_results:
                        print(f"  {grid_pos:10s}: {detection}")

                task_handle = run_task(_sequence_worker, blocking=False)
                state = "RUNNING"

        elif state == "RUNNING":
            if robot.was_button_pressed(Button.BTN_2):
                task_handle.cancel()
                task_handle.wait(timeout=AIM_TIMEOUT_S + 1.0)
                task_handle = None
                dim_all_leds(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE -- cancelled, press BTN_1 to repeat")
                state = "IDLE"
            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                dim_all_leds(robot)
                show_idle_leds(robot)
                print("[FSM] IDLE -- scan complete, press BTN_1 to repeat")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
