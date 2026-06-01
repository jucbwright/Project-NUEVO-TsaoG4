"""
start.py — green-light go trigger
===================================
Drop-in replacement for the BTN_1 check in the IDLE state.
Watches for a green traffic light via the vision node, mirrors the detected
color to the onboard LEDs (like traffic_light_leds.py), and returns True the
moment a green light is detected.

Usage in main.py
----------------
    from robot.JuliaFiles.start import LookForGreen, GreenGo

    # call once — add to configure_robot() or the INIT state:
    LookForGreen(robot)

    # replace was_button_pressed(Button.BTN_1) in your IDLE branch:
    elif state == "IDLE":
        if GreenGo(robot):                   # True once green light is seen
            reset_mission_pose(robot)
            show_moving_leds(robot)
            drive_handle = start_path(robot)
            state = "MOVING"
"""
from __future__ import annotations

import time

from robot.hardware_map import LED
from robot.robot import Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

MIN_CONFIDENCE   = 0.50   # ignore detections below this confidence
VISION_STALE_SEC = 3.0    # treat vision as inactive if no frame within this window
LED_BRIGHTNESS   = 255
LIGHT_HOLD_SEC   = 2.0    # keep the LED on this long after last detection

# ---------------------------------------------------------------------------
# Module-level LED hold state (persists between ticks)
# ---------------------------------------------------------------------------

_lights_off_at: float     = 0.0
_last_color:    str | None = None


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def LookForGreen(robot: Robot) -> None:
    """Enable the vision node subscription. Call once during setup."""
    robot.enable_vision()
    robot.set_led(LED.RED,   0)
    robot.set_led(LED.GREEN, 0)


def GreenGo(robot: Robot) -> bool:
    """
    Call once per FSM tick from the IDLE state.

    - Reads traffic-light detections from the vision node.
    - Mirrors the detected color (red/green) to the onboard LEDs.
    - Turns LEDs off if no detection for LIGHT_HOLD_SEC seconds.
    - Returns True as long as a green light is currently detected.

    Replaces was_button_pressed(Button.BTN_1) in the IDLE branch.
    """
    global _lights_off_at, _last_color
    now = time.monotonic()

    color = _find_traffic_light_color(robot)

    if color in ("red", "green"):
        _show_led(robot, color)
        _lights_off_at = now + LIGHT_HOLD_SEC
        if color != _last_color:
            print(f"[start] traffic light: {color}")
        _last_color = color

    elif _lights_off_at > 0.0 and now >= _lights_off_at:
        robot.set_led(LED.RED,   0)
        robot.set_led(LED.GREEN, 0)
        _lights_off_at = 0.0
        if _last_color is not None:
            print("[start] no recent traffic light — watching")
        _last_color = None

    return color == "green"


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------

def _find_traffic_light_color(robot: Robot) -> str | None:
    """Return the highest-confidence red/green traffic-light detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_conf  = -1.0

    for det in robot.get_detections("traffic light"):
        conf = float(det["confidence"])
        if conf < MIN_CONFIDENCE:
            continue
        color = det.get("attributes", {}).get("color", {}).get("value")
        if color not in ("red", "green"):
            continue
        if conf > best_conf:
            best_conf  = conf
            best_color = str(color)

    return best_color


def _show_led(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED,   LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED,   0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)
