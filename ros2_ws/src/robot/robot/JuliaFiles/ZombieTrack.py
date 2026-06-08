"""
zombieTracker.py — pan + tilt STEPPER_1/STEPPER_2 to keep a zombie centred in frame
=====================================================================================

HOW TO RUN
----------
Start the vision node in debug mode first (saves latest.jpg for monitoring):
    ros2 launch vision vision_debug.launch.py

Then copy this file over main.py and restart the robot node:
    cp JuliaFiles/zombieTracker.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Detects zombies via the vision node.
STEPPER_1 pans left/right and STEPPER_2 tilts up/down to keep the zombie
bounding box centred in the camera frame.
Green LED       = zombie visible and being tracked.
All LEDs        = zombie has been centred in frame for STILL_SEC seconds.
Orange LED      = scanning, no zombie in frame.

STILLNESS DETECTION
-------------------
Stillness is measured by how long BOTH the pan error and tilt error stay within
DEADZONE_PX simultaneously. This avoids the pixel-position approach where every
camera correction would reset the timer.

TUNING
------
PAN_STEPS_PER_PIXEL    proportional gain for pan axis.
TILT_STEPS_PER_PIXEL   proportional gain for tilt axis (~1 step = 0.75px on this mechanism).
DEADZONE_PX            no-move zone around frame centre; also the stillness threshold.
PAN_MAX_STEPS_PER_MOVE upper bound on a single pan command.
TILT_MAX_STEPS_PER_MOVE upper bound on a single tilt command.
PAN_MIN / PAN_MAX      soft travel limits for pan axis (steps from startup).
TILT_MIN / TILT_MAX    soft travel limits for tilt axis (steps from startup).
STILL_SEC              seconds both axes must be centred before all LEDs light up.
"""

from __future__ import annotations

import time

# ── DEBUG CAPTURE (comment out this import block to disable) ──────────────────
import os as _os
import shutil as _shutil
# ─────────────────────────────────────────────────────────────────────────────

from robot.hardware_map import (
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
    StepMoveType,
    Stepper,
    StepperMotionState,
)
from robot.robot import FirmwareState, Robot

ALL_LEDS = (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE)

# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------

PAN_STEPPER  = Stepper.STEPPER_1
TILT_STEPPER = Stepper.STEPPER_2

PAN_MAX_VEL  = 200   # steps/s
PAN_ACCEL    = 150   # steps/s²
TILT_MAX_VEL = 120   # steps/s  (matches ZombieTilt)
TILT_ACCEL   =  80   # steps/s² (matches ZombieTilt)

# ---------------------------------------------------------------------------
# Tracking parameters
# ---------------------------------------------------------------------------

MIN_CONFIDENCE   = 0.30
VISION_STALE_SEC = 3.0

DEADZONE_PX = 30   # pixels either side of centre — no correction AND stillness threshold

# Pan axis (working values — keep as-is)
PAN_STEPS_PER_PIXEL    = 0.15
PAN_MAX_STEPS_PER_MOVE = 15
PAN_COOLDOWN_SEC       = 0.6
PAN_MIN = -500
PAN_MAX =  500

# Tilt axis (calibrated from ZombieTilt: ~1 step = 0.75px on this mechanism)
TILT_STEPS_PER_PIXEL    = 0.50
TILT_MAX_STEPS_PER_MOVE = 5
TILT_COOLDOWN_SEC       = 1.50
TILT_MIN = -100
TILT_MAX =  100

LED_BRIGHTNESS = 255
HOLD_SEC       = 2.0   # keep green LED on this long after the last sighting

STILL_SEC = 3.0   # seconds both errors must stay within DEADZONE_PX before all LEDs light up

# ── DEBUG CAPTURE — comment out to disable ───────────────────────────────────
_DBG_DIR        = "/runtime_output/julia_debug"   # host: ros2_ws/runtime_output/julia_debug/
_DBG_SRC        = "/runtime_output/vision/latest.jpg"
_DBG_FRAMES     = 20
_DBG_INTERVAL_S = 0.5
# ─────────────────────────────────────────────────────────────────────────────


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def _stepper_is_idle(robot: Robot, stepper: Stepper) -> bool:
    """Return True when the stepper is not actively moving.
    Only ACCEL/CRUISE/DECEL block a new command; IDLE, FAULT, and unknown
    states are treated as safe-to-command so a stale firmware state never
    silently freezes the tracker.
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


# ---------------------------------------------------------------------------
# run()
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state         = "INIT"
    pan_pos       = 0
    tilt_pos      = 0
    zombie_off_at = 0.0
    zombie_active = False

    pan_next_allowed  = 0.0
    tilt_next_allowed = 0.75  # offset so pan and tilt never fire on the same tick

    # ── DEBUG CAPTURE ──────────────────────────────────────────────────────────
    _dbg_idx      = 0
    _dbg_next_cap = 0.0
    # ──────────────────────────────────────────────────────────────────────────

    # Stillness: timer runs only while both pan and tilt errors are within deadzone.
    still_since  = None
    all_leds_lit = False

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            # Match STEPmanipulation_JW.py exactly — no disable, just config + enable.
            # step_disable was breaking things; STEPmanipulation proved this sequence works.
            robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
            robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
            robot.step_enable(PAN_STEPPER)
            robot.step_enable(TILT_STEPPER)
            # ── DEBUG CAPTURE ──────────────────────────────────────────────
            _os.makedirs(_DBG_DIR, exist_ok=True)
            # ──────────────────────────────────────────────────────────────
            _dim_all_leds(robot)
            robot.set_led(LED.ORANGE, 200)
            print("[FSM] SCANNING — pan+tilt tracking zombies")
            state = "SCANNING"

        elif state == "SCANNING":
            now    = time.monotonic()
            zombie = _find_zombie(robot)

            if zombie is not None:
                zombie_off_at = now + HOLD_SEC

                if not zombie_active:
                    zombie_active = True
                    print("[ZOMBIE] Target acquired — tracking")

                # ── DEBUG CAPTURE ──────────────────────────────────────────
                if now >= _dbg_next_cap:
                    try:
                        dst = f"{_DBG_DIR}/frame_{_dbg_idx:02d}.jpg"
                        _shutil.copy2(_DBG_SRC, dst)
                        _dbg_idx = (_dbg_idx + 1) % _DBG_FRAMES
                        _dbg_next_cap = now + _DBG_INTERVAL_S
                    except Exception:
                        pass
                # ──────────────────────────────────────────────────────────

                bbox      = zombie["bbox"]
                zombie_cx = bbox["x"] + bbox["width"]  / 2.0
                zombie_cy = bbox["y"] + bbox["height"] / 2.0

                frame_w, frame_h = robot.get_detection_image_size()
                pan_error  = zombie_cx - frame_w / 2.0 if frame_w > 0 else 0.0
                tilt_error = zombie_cy - frame_h / 2.0 if frame_h > 0 else 0.0

                # --- Stillness detection (error-based, both axes) ---
                if abs(pan_error) <= DEADZONE_PX and abs(tilt_error) <= DEADZONE_PX:
                    if still_since is None:
                        still_since = now
                    if not all_leds_lit and (now - still_since) >= STILL_SEC:
                        all_leds_lit = True
                        for led in ALL_LEDS:
                            robot.set_led(led, LED_BRIGHTNESS)
                        print("[ZOMBIE] Standing still — all LEDs on!")
                else:
                    if all_leds_lit:
                        all_leds_lit = False
                        _dim_all_leds(robot)
                        robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                        print("[ZOMBIE] Moving again — tracking")
                    still_since = None

                if not all_leds_lit:
                    robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                    robot.set_led(LED.ORANGE, 0)

                # --- Pan correction (horizontal) ---
                if not all_leds_lit and frame_w > 0 and now >= pan_next_allowed and _stepper_is_idle(robot, PAN_STEPPER):
                    if abs(pan_error) > DEADZONE_PX:
                        steps = int(
                            max(-PAN_MAX_STEPS_PER_MOVE,
                                min(PAN_MAX_STEPS_PER_MOVE, -pan_error * PAN_STEPS_PER_PIXEL))
                        )
                        new_pos = max(PAN_MIN, min(PAN_MAX, pan_pos + steps))
                        steps   = new_pos - pan_pos
                        if steps != 0:
                            robot.step_move(PAN_STEPPER, steps=steps,
                                            move_type=StepMoveType.RELATIVE, blocking=False)
                            pan_pos = new_pos
                            pan_next_allowed = now + PAN_COOLDOWN_SEC
                            direction = "→" if steps > 0 else "←"
                            print(f"[PAN]  {direction} {steps:+d} steps  "
                                  f"(err={pan_error:+.0f}px  pos={pan_pos})")

                # --- Tilt correction (vertical) ---
                if not all_leds_lit and frame_h > 0 and now >= tilt_next_allowed and _stepper_is_idle(robot, TILT_STEPPER):
                    if abs(tilt_error) > DEADZONE_PX:
                        steps = int(
                            max(-TILT_MAX_STEPS_PER_MOVE,
                                min(TILT_MAX_STEPS_PER_MOVE, -tilt_error * TILT_STEPS_PER_PIXEL))
                        )
                        new_pos = max(TILT_MIN, min(TILT_MAX, tilt_pos + steps))
                        steps   = new_pos - tilt_pos
                        if steps != 0:
                            robot.step_move(TILT_STEPPER, steps=steps,
                                            move_type=StepMoveType.RELATIVE, blocking=False)
                            tilt_pos = new_pos
                            tilt_next_allowed = now + TILT_COOLDOWN_SEC
                            direction = "↑" if steps > 0 else "↓"
                            print(f"[TILT] {direction} {steps:+d} steps  "
                                  f"(err={tilt_error:+.0f}px  pos={tilt_pos})")

            elif zombie_off_at > 0.0 and now >= zombie_off_at:
                _dim_all_leds(robot)
                robot.set_led(LED.ORANGE, 200)
                zombie_off_at = 0.0
                still_since   = None
                all_leds_lit  = False
                if zombie_active:
                    zombie_active = False
                    print("[ZOMBIE] Lost target — scanning")

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
