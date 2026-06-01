"""
stop.py — stop-sign proximity trigger
=======================================
Uses the camera bounding-box width as a monocular distance proxy.
A stop sign that fills more of the frame is closer to the robot.
When the detected bounding-box width crosses STOP_SIGN_WIDTH_THRESHOLD,
StopSignNear() returns True and the caller should stop the robot.

How the distance estimation works
----------------------------------
The vision node reports detection["width"] as a fraction of the frame width
(0.0 – 1.0). Because a stop sign has a fixed real-world size, a larger
fraction means the robot is closer. No depth sensor is required.

Tune STOP_SIGN_WIDTH_THRESHOLD by running the robot toward a stop sign and
watching the "[stop] stop sign width=" prints until the robot is at the
distance you want to stop at, then set the threshold to that value.

Usage in main.py
----------------
    from robot.JuliaFiles.stop import LookForStop, StopSignNear

    # call once — add to configure_robot() or the INIT state:
    LookForStop(robot)

    # in the MOVING branch, check every tick:
    elif state == "MOVING":
        if StopSignNear(robot):              # stop sign is close enough
            drive_handle.cancel()
            drive_handle.wait(timeout=1.0)
            drive_handle = None
            robot.stop()
            show_idle_leds(robot)
            print("[FSM] IDLE — stopped for stop sign")
            state = "IDLE"
        elif drive_handle is not None and drive_handle.is_finished():
            ...
"""
from __future__ import annotations

import time

from robot.hardware_map import LED
from robot.robot import Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

MIN_CONFIDENCE = 0.50   # ignore detections below this confidence

# Bounding-box width fraction (0.0–1.0) at which the robot should stop.
# 0.20 ≈ stop sign occupies ~20% of frame width — roughly 0.5–1 m away
# depending on your camera FOV. Increase to stop closer, decrease to stop
# farther away. Print statements below will help you tune this.
STOP_SIGN_WIDTH_THRESHOLD = 0.20

VISION_STALE_SEC = 3.0
LED_BRIGHTNESS   = 255
LED_HOLD_SEC     = 2.0

# ---------------------------------------------------------------------------
# Module-level LED hold state (persists between ticks)
# ---------------------------------------------------------------------------

_led_off_at:     float     = 0.0
_last_triggered: bool      = False


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def LookForStop(robot: Robot) -> None:
    """Enable the vision node subscription. Call once during setup."""
    robot.enable_vision()
    robot.set_led(LED.RED, 0)


def StopSignNear(robot: Robot) -> bool:
    """
    Call once per FSM tick from the MOVING state.

    - Reads stop-sign detections from the vision node.
    - Lights the red LED when a stop sign is in frame.
    - Turns the LED off if no detection for LED_HOLD_SEC seconds.
    - Prints bounding-box width each tick so you can tune the threshold.
    - Returns True when the best detection's width >= STOP_SIGN_WIDTH_THRESHOLD.

    Replaces (or augments) the BTN_2 cancel check in the MOVING branch.
    """
    global _led_off_at, _last_triggered
    now = time.monotonic()

    width, confidence = _find_best_stop_sign(robot)

    if width is not None:
        print(f"[stop] stop sign width={width:.3f}  confidence={confidence:.2f}  "
              f"threshold={STOP_SIGN_WIDTH_THRESHOLD:.3f}")
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        _led_off_at = now + LED_HOLD_SEC
        triggered = width >= STOP_SIGN_WIDTH_THRESHOLD
        if triggered and not _last_triggered:
            print(f"[stop] STOP — sign width {width:.3f} >= threshold {STOP_SIGN_WIDTH_THRESHOLD:.3f}")
        _last_triggered = triggered
        return triggered

    # no detection this tick
    if _led_off_at > 0.0 and now >= _led_off_at:
        robot.set_led(LED.RED, 0)
        _led_off_at = 0.0

    _last_triggered = False
    return False


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _find_best_stop_sign(robot: Robot):
    """
    Return (width, confidence) for the highest-confidence stop-sign detection,
    or (None, None) if none found.
    """
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None, None

    best_width = None
    best_conf  = -1.0

    for det in robot.get_detections("stop sign"):
        conf = float(det["confidence"])
        if conf < MIN_CONFIDENCE:
            continue
        if conf > best_conf:
            best_conf  = conf
            best_width = float(det["width"])

    return best_width, best_conf
