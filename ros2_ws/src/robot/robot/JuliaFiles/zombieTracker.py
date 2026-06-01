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
Detects "persons" classified as green (zombies) via the vision node.
STEPPER_1 pans left/right and STEPPER_2 tilts up/down to keep the zombie
bounding box centred in the camera frame.
Green LED       = zombie visible and being tracked.
All LEDs        = zombie has been standing still for STILL_SEC seconds.
Orange LED      = scanning, no zombie in frame.

TUNING
------
STEPS_PER_PIXEL      proportional gain — steps issued per pixel of error.
                     Increase if the tracker feels sluggish; decrease if it overshoots.
DEADZONE_PX          half-width of the no-move zone around frame centre (pixels).
                     Keeps steppers still when the zombie is already close to centre.
MAX_STEPS_PER_MOVE   upper bound on a single pan/tilt command — prevents slamming.
PAN_MIN / PAN_MAX    soft travel limits for pan axis (steps from startup).
TILT_MIN / TILT_MAX  soft travel limits for tilt axis (steps from startup).
STILL_TOLERANCE_PX   max pixel drift between frames to still count as "not moving".
STILL_SEC            seconds the zombie must be still before all LEDs light up.
"""

from __future__ import annotations

import math
import time

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
TILT_MAX_VEL = 200   # steps/s
TILT_ACCEL   = 150   # steps/s²

# ---------------------------------------------------------------------------
# Tracking parameters  (tune these to your mechanism)
# ---------------------------------------------------------------------------

MIN_CONFIDENCE    = 0.30   # ignore detections below this confidence
VISION_STALE_SEC  = 3.0    # treat vision as offline after this many seconds without a frame

DEADZONE_PX        = 30    # pixels either side of centre → no correction issued
STEPS_PER_PIXEL    = 0.15  # proportional gain (small = gentle nudges)
MAX_STEPS_PER_MOVE = 15    # clamp per-command — keeps each move a tiny increment
PAN_COOLDOWN_SEC   = 0.6   # minimum seconds between pan commands
TILT_COOLDOWN_SEC  = 0.6   # minimum seconds between tilt commands

PAN_MIN  = -500   # leftmost  allowed absolute pan  position (steps from startup)
PAN_MAX  =  500   # rightmost allowed absolute pan  position (steps from startup)
TILT_MIN = -200   # lowest    allowed absolute tilt position (steps from startup)
TILT_MAX =  200   # highest   allowed absolute tilt position (steps from startup)

LED_BRIGHTNESS    = 255
HOLD_SEC          = 2.0    # keep green LED on this long after the last sighting

STILL_TOLERANCE_PX = 25    # max pixel drift between frames to count as "not moving"
STILL_SEC          = 3.0   # seconds standing still before all LEDs light up


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def _stepper_is_idle(robot: Robot, stepper: Stepper) -> bool:
    """Return True when the given stepper is not currently moving."""
    step_state = robot.get_step_state()
    if step_state is None:
        return True
    idx = int(stepper) - 1
    return step_state.steppers[idx].motion_state == int(StepperMotionState.IDLE)


def _find_zombie(robot: Robot) -> dict | None:
    """Return the highest-confidence green-person detection, or None."""
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
    pan_pos       = 0      # running tally of absolute pan  steps from startup
    tilt_pos      = 0      # running tally of absolute tilt steps from startup
    zombie_off_at = 0.0
    zombie_active = False

    pan_next_allowed  = 0.0  # earliest time the next pan  command may fire
    tilt_next_allowed = 0.0  # earliest time the next tilt command may fire

    # Stillness tracking
    still_ref_cx  = None   # zombie centre-X when stillness timer last reset
    still_ref_cy  = None   # zombie centre-Y when stillness timer last reset
    still_since   = None   # time.monotonic() when zombie stopped moving significantly
    all_leds_lit  = False  # True while the all-LEDs alert is active

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

                bbox      = zombie["bbox"]
                zombie_cx = bbox["x"] + bbox["width"]  / 2.0
                zombie_cy = bbox["y"] + bbox["height"] / 2.0

                # --- Stillness detection ---
                if still_ref_cx is None:
                    still_ref_cx, still_ref_cy = zombie_cx, zombie_cy
                    still_since = now
                else:
                    dist = math.hypot(zombie_cx - still_ref_cx, zombie_cy - still_ref_cy)
                    if dist > STILL_TOLERANCE_PX:
                        still_ref_cx, still_ref_cy = zombie_cx, zombie_cy
                        still_since = now
                        if all_leds_lit:
                            all_leds_lit = False
                            _dim_all_leds(robot)
                            robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                            print("[ZOMBIE] Moving again — tracking")

                if not all_leds_lit and still_since is not None and (now - still_since) >= STILL_SEC:
                    all_leds_lit = True
                    for led in ALL_LEDS:
                        robot.set_led(led, LED_BRIGHTNESS)
                    print("[ZOMBIE] Standing still — all LEDs on!")

                if not all_leds_lit:
                    robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                    robot.set_led(LED.ORANGE, 0)

                frame_w, frame_h = robot.get_detection_image_size()

                # --- Pan correction (horizontal, skip while zombie confirmed still) ---
                if not all_leds_lit and frame_w > 0 and now >= pan_next_allowed and _stepper_is_idle(robot, PAN_STEPPER):
                    pan_error = zombie_cx - frame_w / 2.0   # + = zombie is right of centre

                    if abs(pan_error) > DEADZONE_PX:
                        steps = int(
                            max(-MAX_STEPS_PER_MOVE,
                                min(MAX_STEPS_PER_MOVE, -pan_error * STEPS_PER_PIXEL))
                        )
                        new_pos = max(PAN_MIN, min(PAN_MAX, pan_pos + steps))
                        steps   = new_pos - pan_pos

                        if steps != 0:
                            robot.step_move(
                                PAN_STEPPER,
                                steps=steps,
                                move_type=StepMoveType.RELATIVE,
                                blocking=False,
                            )
                            pan_pos = new_pos
                            pan_next_allowed = now + PAN_COOLDOWN_SEC
                            direction = "→" if steps > 0 else "←"
                            print(f"[PAN]  {direction} {steps:+d} steps  "
                                  f"(err={pan_error:+.0f}px  pos={pan_pos})")

                # --- Tilt correction (vertical, skip while zombie confirmed still) ---
                if not all_leds_lit and frame_h > 0 and now >= tilt_next_allowed and _stepper_is_idle(robot, TILT_STEPPER):
                    tilt_error = zombie_cy - frame_h / 2.0  # + = zombie is below centre

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
                zombie_off_at  = 0.0
                still_ref_cx   = None
                still_ref_cy   = None
                still_since    = None
                all_leds_lit   = False
                if zombie_active:
                    zombie_active = False
                    print("[ZOMBIE] Lost target — scanning")

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
