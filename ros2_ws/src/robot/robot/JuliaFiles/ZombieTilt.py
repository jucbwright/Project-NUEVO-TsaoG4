"""
ZombieTilt.py — tilt STEPPER_2 to keep a zombie vertically centred in frame
=============================================================================

HOW TO RUN
----------
Start the vision node in debug mode first (saves latest.jpg for monitoring):
    ros2 launch vision vision_debug.launch.py

Then copy this file over main.py and restart the robot node:
    cp JuliaFiles/ZombieTilt.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Detects zombies via the vision node.
STEPPER_2 tilts up/down to keep the zombie bounding box vertically centred
in the camera frame.
Green LED       = zombie visible and being tracked.
All LEDs        = zombie has been centred in frame for STILL_SEC seconds.
Orange LED      = scanning, no zombie in frame.

STILLNESS DETECTION
-------------------
Stillness is measured by how long the vertical error stays within DEADZONE_PX.
This is intentional: raw pixel position shifts every time the camera tilts,
so tracking absolute pixel position would reset the timer on every correction.
Measuring error instead means the timer only runs when the zombie is actually
centred — i.e., the tilt has settled and the person is not moving.

TUNING
------
STEPS_PER_PIXEL    proportional gain — steps issued per pixel of vertical error.
DEADZONE_PX        no-tilt zone around frame centre; also the stillness threshold.
MAX_STEPS_PER_MOVE upper bound on a single tilt command — prevents slamming.
TILT_MIN / TILT_MAX soft travel limits in absolute steps from the startup position.
STILL_SEC          seconds the zombie must be centred before all LEDs light up.
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

TILT_STEPPER = Stepper.STEPPER_2

TILT_MAX_VEL = 120   # steps/s
TILT_ACCEL   =  80   # steps/s²

# ---------------------------------------------------------------------------
# Tracking parameters  (tune these to your mechanism)
# ---------------------------------------------------------------------------

MIN_CONFIDENCE    = 0.30   # ignore detections below this confidence
VISION_STALE_SEC  = 3.0    # treat vision as offline after this many seconds without a frame

DEADZONE_PX        = 30    # pixels either side of vertical centre — no correction AND stillness threshold
STEPS_PER_PIXEL    = 0.50  # proportional gain — calibrated: ~1 step = 0.75px on this mechanism
MAX_STEPS_PER_MOVE =  5    # clamp per-command — prevents slamming
TILT_COOLDOWN_SEC  = 1.50  # minimum seconds between tilt commands

TILT_MIN = -100   # lowest  allowed absolute tilt position (steps from startup)
TILT_MAX =  100   # highest allowed absolute tilt position (steps from startup)

LED_BRIGHTNESS    = 255
HOLD_SEC          = 2.0    # keep green LED on this long after the last sighting

STILL_SEC = 3.0   # seconds the error must stay within DEADZONE_PX before all LEDs light up

# ── DEBUG CAPTURE — comment out to disable ───────────────────────────────────
_DBG_DIR        = "/runtime_output/julia_debug"   # host: ros2_ws/runtime_output/julia_debug/
_DBG_SRC        = "/runtime_output/vision/latest.jpg"
_DBG_FRAMES     = 20       # rolling buffer size (files are overwritten each run)
_DBG_INTERVAL_S = 0.5      # seconds between captures
# ─────────────────────────────────────────────────────────────────────────────


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def _tilt_is_idle(robot: Robot) -> bool:
    """Return True when the tilt stepper is not actively moving.
    Only ACCEL/CRUISE/DECEL block a new command; IDLE, FAULT, and unknown
    states are treated as safe-to-command so a stale or unexpected firmware
    state never silently freezes the tracker.
    """
    step_state = robot.get_step_state()
    if step_state is None:
        return True
    idx = int(TILT_STEPPER) - 1
    motion = step_state.steppers[idx].motion_state
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
    tilt_pos      = 0      # running tally of absolute tilt steps from startup
    zombie_off_at = 0.0
    zombie_active = False

    tilt_next_allowed = 0.0  # earliest time the next tilt command may fire

    # ── DEBUG CAPTURE ──────────────────────────────────────────────────────────
    _dbg_idx      = 0
    _dbg_next_cap = 0.0
    # ──────────────────────────────────────────────────────────────────────────

    # Stillness tracking — based on error staying within deadzone, not pixel position,
    # so camera corrections don't reset the timer.
    still_since  = None   # time.monotonic() when error first entered the deadzone
    all_leds_lit = False  # True while the all-LEDs alert is active

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            # Disable then re-enable to clear any fault left from a previous session
            robot.step_disable(TILT_STEPPER)
            time.sleep(0.1)
            robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
            robot.step_enable(TILT_STEPPER)
            time.sleep(0.1)
            # ── DEBUG CAPTURE ──────────────────────────────────────────────
            _os.makedirs(_DBG_DIR, exist_ok=True)
            # ──────────────────────────────────────────────────────────────
            _dim_all_leds(robot)
            robot.set_led(LED.ORANGE, 200)
            print("[FSM] SCANNING — tilting STEPPER_2 to track zombies vertically")
            state = "SCANNING"

        elif state == "SCANNING":
            now    = time.monotonic()
            zombie = _find_zombie(robot)

            if zombie is not None:
                zombie_off_at = now + HOLD_SEC

                if not zombie_active:
                    zombie_active = True
                    print("[ZOMBIE] Target acquired — tilt tracking")

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
                zombie_cy = bbox["y"] + bbox["height"] / 2.0

                _, frame_h = robot.get_detection_image_size()
                tilt_error = zombie_cy - frame_h / 2.0 if frame_h > 0 else 0.0

                # --- Stillness detection (error-based) ---
                # Timer runs while zombie is centred; resets the moment it leaves deadzone.
                if abs(tilt_error) <= DEADZONE_PX:
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

                # --- Tilt correction (skip while zombie is confirmed centred) ---
                print(f"[TILT DBG] err={tilt_error:+.0f}px  still={still_since is not None}  "
                      f"idle={_tilt_is_idle(robot)}  cooldown={max(0.0, tilt_next_allowed - now):.2f}s")
                if not all_leds_lit and frame_h > 0 and now >= tilt_next_allowed and _tilt_is_idle(robot):
                    if abs(tilt_error) > DEADZONE_PX:
                        steps = int(
                            max(-MAX_STEPS_PER_MOVE,
                                min(MAX_STEPS_PER_MOVE, -tilt_error * STEPS_PER_PIXEL))
                        )
                        new_pos = max(TILT_MIN, min(TILT_MAX, tilt_pos + steps))
                        steps   = new_pos - tilt_pos

                        if steps != 0:
                            robot.step_move(
                                TILT_STEPPER,
                                steps=steps,
                                move_type=StepMoveType.RELATIVE,
                                blocking=False,
                            )
                            tilt_pos = new_pos
                            tilt_next_allowed = now + TILT_COOLDOWN_SEC
                            direction = "↑" if steps > 0 else "↓"
                            print(f"[TILT] {direction} {steps:+d} steps  "
                                  f"(err={tilt_error:+.0f}px  pos={tilt_pos})")

            elif zombie_off_at > 0.0 and now >= zombie_off_at:
                # Zombie gone long enough — reset everything
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
