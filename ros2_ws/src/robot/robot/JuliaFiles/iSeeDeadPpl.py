"""
iSeeDeadPpl.py - detect a "zombie" (a person that is green)
===============================================================

HOW TO RUN
----------
Start the vision node in another terminal:
    ros2 run vision vision_node

Then copy this file over main.py and restart the robot node:
    cp JuliaFiles/iSeeDeadPpl.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Scans detections for objects classified as "person".
For each person, checks if their color attribute is "green".
If a green person (zombie) is found, the green LED lights up and
a message is printed. If no zombie is seen for HOLD_SEC seconds,
the LED turns off.

WHAT THIS TEACHES
-----------------
1. Calling get_detections() with a specific class name
2. Reading an attribute (color) from a detection
3. Combining class + attribute to form a compound condition
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS   = 255
HOLD_SEC         = 2.0    # how long to keep LED on after last sighting
VISION_STALE_SEC = 3.0    # how long before vision is considered stale
MIN_CONFIDENCE   = 0.50   # ignore detections below this confidence


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def find_zombie(robot: Robot) -> dict | None:
    """Return the highest-confidence green person detection, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best = None
    best_conf = -1.0

    for detection in robot.get_detections("person"):
        confidence = float(detection["confidence"])
        if confidence < MIN_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attr = attributes.get("color", {})
        color      = color_attr.get("value")

        if color != "green":
            continue

        if confidence > best_conf:
            best_conf = confidence
            best = detection

    return best


# ---------------------------------------------------------------------------
# run() - entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state         = "INIT"
    zombie_off_at = 0.0
    zombie_active = False

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            dim_all_leds(robot)
            print("[FSM] SCANNING - looking for green persons (zombies)")
            state = "SCANNING"

        elif state == "SCANNING":
            now    = time.monotonic()
            zombie = find_zombie(robot)

            if zombie is not None:
                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                zombie_off_at = now + HOLD_SEC

                if not zombie_active:
                    zombie_active = True
                    x    = zombie.get("x", "?")
                    y    = zombie.get("y", "?")
                    w    = zombie.get("width", "?")
                    h    = zombie.get("height", "?")
                    conf = zombie.get("confidence", "?")
                    print(f"[ZOMBIE] Target acquired! "
                          f"x={x} y={y} w={w} h={h} conf={conf:.2f}")

            elif zombie_off_at > 0.0 and now >= zombie_off_at:
                robot.set_led(LED.GREEN, 0)
                zombie_off_at = 0.0
                if zombie_active:
                    zombie_active = False
                    print("[ZOMBIE] Lost target — scanning...")

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
