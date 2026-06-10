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
    StepperMotionState,
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
# Centering (same approach/values as zombieTracker.py)
# ---------------------------------------------------------------------------

DEADZONE_PX = 30   # pixels either side of frame centre — no correction AND "centred" threshold

PAN_STEPS_PER_PIXEL    = 0.15
PAN_MAX_STEPS_PER_MOVE = 15
PAN_COOLDOWN_SEC       = 0.6
PAN_MIN = -500
PAN_MAX =  500

TILT_STEPS_PER_PIXEL    = 0.50
TILT_MAX_STEPS_PER_MOVE = 5
TILT_COOLDOWN_SEC       = 1.50
TILT_MIN = -100
TILT_MAX =  100

CENTER_STILL_SEC = 1.0   # seconds the chosen zombie must stay within the deadzone before "centred"


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


def _select_target_zombie(robot: Robot, frame_w: int, frame_h: int) -> dict | None:
    """Among all zombie detections, return the one nearest to the target
    coordinate (the frame centre, i.e. where this grid position is aimed),
    or None if no zombie is detected."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    target_x, target_y = frame_w / 2.0, frame_h / 2.0
    best, best_dist = None, float("inf")
    for det in robot.get_detections("zombie"):
        if float(det["confidence"]) < MIN_CONFIDENCE:
            continue
        bbox = det["bbox"]
        cx = bbox["x"] + bbox["width"] / 2.0
        cy = bbox["y"] + bbox["height"] / 2.0
        dist = (cx - target_x) ** 2 + (cy - target_y) ** 2
        if dist < best_dist:
            best_dist = dist
            best = det
    return best


def _stepper_is_idle(robot: Robot, stepper: Stepper) -> bool:
    """Return True when the stepper is not actively moving.
    Only ACCEL/CRUISE/DECEL block a new command; IDLE, FAULT, and unknown
    states are treated as safe-to-command so a stale firmware state never
    silently freezes the centering correction.
    """
    step_state = robot.get_step_state()
    if step_state is None:
        return True
    motion = step_state.steppers[int(stepper) - 1].motion_state
    return motion not in (
        int(StepperMotionState.ACCEL),
        int(StepperMotionState.CRUISE),
        int(StepperMotionState.DECEL),
    )


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
                        dwell_end = time.monotonic() + DWELL_S
                        zombie_found = False
                        still_since: float | None = None
                        pan_next_allowed = 0.0
                        tilt_next_allowed = 0.0
                        while time.monotonic() < dwell_end:
                            if task.cancelled():
                                return
                            now = time.monotonic()
                            frame_w, frame_h = robot.get_detection_image_size()
                            zombie = _select_target_zombie(robot, frame_w, frame_h)

                            if zombie is None:
                                still_since = None
                                _dim_all_leds(robot)
                                robot.set_led(LED.ORANGE, 200)
                                time.sleep(DWELL_POLL_S)
                                continue

                            bbox = zombie["bbox"]
                            zombie_cx = bbox["x"] + bbox["width"] / 2.0
                            zombie_cy = bbox["y"] + bbox["height"] / 2.0
                            pan_error  = zombie_cx - frame_w / 2.0 if frame_w > 0 else 0.0
                            tilt_error = zombie_cy - frame_h / 2.0 if frame_h > 0 else 0.0
                            centered = abs(pan_error) <= DEADZONE_PX and abs(tilt_error) <= DEADZONE_PX

                            if centered:
                                if still_since is None:
                                    still_since = now
                                if not zombie_found and (now - still_since) >= CENTER_STILL_SEC:
                                    zombie_found = True
                                    print(f"[ZOMBIE] Centered at {target.label}! Holding 2s for shot...")
                                    for led in ALL_LEDS:
                                        robot.set_led(led, LED_BRIGHTNESS)
                            else:
                                still_since = None

                            if not zombie_found:
                                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                                robot.set_led(LED.ORANGE, 0)

                                # --- Pan correction (horizontal) ---
                                if (frame_w > 0 and now >= pan_next_allowed
                                        and _stepper_is_idle(robot, PAN_STEPPER)
                                        and abs(pan_error) > DEADZONE_PX):
                                    steps = int(
                                        max(-PAN_MAX_STEPS_PER_MOVE,
                                            min(PAN_MAX_STEPS_PER_MOVE, -pan_error * PAN_STEPS_PER_PIXEL))
                                    )
                                    new_pos = max(PAN_MIN, min(PAN_MAX, pos["pan"] + steps))
                                    steps   = new_pos - pos["pan"]
                                    if steps != 0:
                                        robot.step_move(PAN_STEPPER, steps=steps,
                                                        move_type=StepMoveType.RELATIVE, blocking=False)
                                        pos["pan"] = new_pos
                                        pan_next_allowed = now + PAN_COOLDOWN_SEC

                                # --- Tilt correction (vertical) ---
                                if (frame_h > 0 and now >= tilt_next_allowed
                                        and _stepper_is_idle(robot, TILT_STEPPER)
                                        and abs(tilt_error) > DEADZONE_PX):
                                    steps = int(
                                        max(-TILT_MAX_STEPS_PER_MOVE,
                                            min(TILT_MAX_STEPS_PER_MOVE, -tilt_error * TILT_STEPS_PER_PIXEL))
                                    )
                                    new_pos = max(TILT_MIN, min(TILT_MAX, pos["tilt"] + steps))
                                    steps   = new_pos - pos["tilt"]
                                    if steps != 0:
                                        robot.step_move(TILT_STEPPER, steps=steps,
                                                        move_type=StepMoveType.RELATIVE, blocking=False)
                                        pos["tilt"] = new_pos
                                        tilt_next_allowed = now + TILT_COOLDOWN_SEC

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
                print("[FSM] IDLE -- press BTN_1 to run the aim sequence")
                state = "IDLE"
            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] DONE -- sweep complete")
                print("[FSM] IDLE -- press BTN_1 to run the aim sequence")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
