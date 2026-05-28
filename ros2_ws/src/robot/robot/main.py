"""
zombie_sweep.py -- grid sweep + green-person (zombie) detection
===============================================================
Sweeps a 3x3 pan/tilt grid. At each position, checks vision for a
"person" with color="green". On first zombie found, stops the sweep,
aims precisely at the detection center, and locks there.

HOW TO RUN
----------
Terminal A:  ros2 launch vision vision_debug.launch.py
Terminal B:  cp JuliaFiles/zombie_sweep.py main.py
             ros2 run robot robot
"""

from __future__ import annotations

import time
from typing import NamedTuple

class Target(NamedTuple):
    label: str
    pan:   int   # absolute steps from startup origin (STEPPER_1)
    tilt:  int   # absolute steps from startup origin (STEPPER_2)

from robot.hardware_map import (
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Hardware config
# ---------------------------------------------------------------------------
PAN_STEPPER  = Stepper.STEPPER_1
TILT_STEPPER = Stepper.STEPPER_2

PAN_MAX_VEL  = 800
PAN_ACCEL    = 400
TILT_MAX_VEL = 600
TILT_ACCEL   = 300
AIM_TIMEOUT_S = 10.0

# How long to pause at each grid point while checking vision (seconds)
VISION_DWELL_S = 2.0

# Vision settings
MIN_CONFIDENCE  = 0.50
VISION_STALE_SEC = 3.0

# Grid positions (steps) — tune to your build
PAN_L,  PAN_C,  PAN_R  = -400,  0,  400
TILT_U, TILT_C, TILT_D =  200,  0, -200

# Camera resolution (used to convert pixel coords → stepper offset)
IMG_W, IMG_H = 640, 480

# Steps-per-pixel scale factors — tune after calibration
# Positive pan  = right,  positive tilt = up
PAN_STEPS_PER_PX  =  1.2   # steps per pixel in X
TILT_STEPS_PER_PX = -1.0   # steps per pixel in Y (Y increases downward)

SEQUENCE: list[Target] = [
    Target("L_UP",   PAN_L, TILT_U),
    Target("L_CTR",  PAN_L, TILT_C),
    Target("L_DOWN", PAN_L, TILT_D),
    Target("C_DOWN", PAN_C, TILT_D),
    Target("C_CTR",  PAN_C, TILT_C),
    Target("C_UP",   PAN_C, TILT_U),
    Target("R_UP",   PAN_R, TILT_U),
    Target("R_CTR",  PAN_R, TILT_C),
    Target("R_DOWN", PAN_R, TILT_D),
    Target("HOME",   PAN_C, TILT_C),
]

# ---------------------------------------------------------------------------
# Stepper state
# ---------------------------------------------------------------------------
_current_pan:  int = 0
_current_tilt: int = 0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def setup_steppers(robot: Robot) -> None:
    global _current_pan, _current_tilt
    robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
    robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
    robot.step_enable(PAN_STEPPER)
    robot.step_enable(TILT_STEPPER)
    _current_pan = _current_tilt = 0

def aim_at(robot: Robot, target: Target) -> bool:
    global _current_pan, _current_tilt
    print(f"[AIM] -> {target.label}  pan={target.pan}  tilt={target.tilt}")
    if target.pan != _current_pan:
        ok = robot.step_move(PAN_STEPPER, steps=target.pan,
                             move_type=StepMoveType.ABSOLUTE,
                             blocking=True, timeout=AIM_TIMEOUT_S)
        if not ok:
            print(f"[warn] pan timed out at {target.label}")
            return False
        _current_pan = target.pan

    if target.tilt != _current_tilt:
        ok = robot.step_move(TILT_STEPPER, steps=target.tilt,
                             move_type=StepMoveType.ABSOLUTE,
                             blocking=True, timeout=AIM_TIMEOUT_S)
        if not ok:
            print(f"[warn] tilt timed out at {target.label}")
            return False
        _current_tilt = target.tilt

    return True

def find_zombie(robot: Robot) -> dict | None:
    """Return the highest-confidence green person, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    best      = None
    best_conf = -1.0
    for detection in robot.get_detections("person"):
        conf = float(detection["confidence"])
        if conf < MIN_CONFIDENCE:
            continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color != "green":
            continue
        if conf > best_conf:
            best_conf = conf
            best = detection
    return best

def pixel_to_step_offset(detection: dict) -> tuple[int, int]:
    """
    Convert detection bounding-box center to stepper offsets
    relative to the current position.
    """
    cx = detection["bbox"]["x"] + detection["bbox"]["width"]  // 2
    cy = detection["bbox"]["y"] + detection["bbox"]["height"] // 2
    dx = cx - IMG_W // 2   # pixels right of image center
    dy = cy - IMG_H // 2   # pixels below image center
    pan_offset  = int(dx * PAN_STEPS_PER_PX)
    tilt_offset = int(dy * TILT_STEPS_PER_PX)
    return pan_offset, tilt_offset

# ---------------------------------------------------------------------------
# FSM entry point
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state     = "INIT"
    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # -- INIT -----------------------------------------------------------
        if state == "INIT":
            start_robot(robot)
            setup_steppers(robot)
            robot.set_led(LED.ORANGE, 200)
            robot.set_led(LED.GREEN, 0)
            print("[FSM] IDLE -- press BTN_1 to start zombie sweep")
            state = "IDLE"

        # -- IDLE -----------------------------------------------------------
        elif state == "IDLE":
            from robot.hardware_map import Button
            if robot.was_button_pressed(Button.BTN_1):
                robot.set_led(LED.ORANGE, 0)
                robot.set_led(LED.GREEN, 200)
                print("[FSM] SWEEP -- scanning grid for zombies")
                state = "SWEEP"

        # -- SWEEP ----------------------------------------------------------
        elif state == "SWEEP":
            zombie_found = False

            for target in SEQUENCE:
                aim_at(robot, target)

                # Check vision during dwell window
                dwell_end = time.monotonic() + VISION_DWELL_S
                while time.monotonic() < dwell_end:
                    zombie = find_zombie(robot)
                    if zombie is not None:
                        print(f"[ZOMBIE] Detected at {target.label}! "
                              f"conf={zombie['confidence']:.2f} "
                              f"x={zombie['bbox']['x']} y={zombie['bbox']['y']} "
                              f"w={zombie['bbox']['width']} h={zombie['bbox']['height']}")
                        zombie_found = True
                        break
                    time.sleep(0.1)

                if zombie_found:
                    # Fine-aim: nudge steppers so zombie is centered in frame
                    pan_off, tilt_off = pixel_to_step_offset(zombie)
                    fine_target = Target(
                        "ZOMBIE",
                        _current_pan  + pan_off,
                        _current_tilt + tilt_off,
                    )
                    aim_at(robot, fine_target)
                    state = "LOCKED"
                    break

            if not zombie_found:
                robot.set_led(LED.GREEN, 0)
                robot.set_led(LED.ORANGE, 200)
                print("[FSM] IDLE -- sweep complete, no zombie found")
                state = "IDLE"

        # -- LOCKED ---------------------------------------------------------
        elif state == "LOCKED":
            # Hold position; blink green to indicate target locked
            robot.set_led(LED.GREEN, 255)
            print("[FSM] LOCKED on zombie -- press BTN_1 to sweep again")

            from robot.hardware_map import Button
            # Wait for button press to reset
            while not robot.was_button_pressed(Button.BTN_1):
                time.sleep(0.1)

            robot.set_led(LED.GREEN, 0)
            robot.set_led(LED.ORANGE, 200)
            print("[FSM] IDLE -- resetting")
            state = "IDLE"

        # -- Tick-rate control ----------------------------------------------
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()