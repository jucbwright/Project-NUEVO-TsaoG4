"""
main.py — student entry point
==============================
This is the only file students are expected to edit.

The structure is intentionally simple:
- keep one plain `state` variable
- write helper functions for robot actions
- use `if state == "..."` inside the main loop

To run:
    ros2 run robot robot
"""

from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


def configure_robot(robot: Robot) -> None:
    """Apply the user unit plus robot-specific wheel mapping and odometry settings."""
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter_mm=WHEEL_DIAMETER,
        wheel_base_mm=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.RED, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    """Start the firmware and reset odometry before the main mission begins."""
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)
    path_control_points = [
        (0.0, 0.0),
        (0.0, 500.0),
        (500.0, 500.0),
        (500.0, 0.0),
        (0.0, 0.0),
    ]
    
    # path = path_control_points
    path = densify_polyline(path_control_points, spacing=25.0)

    state = "INIT"
    drive_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] IDLE (odometry reset)")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            
            if robot.get_button(Button.BTN_1):
                drive_handle = robot.purepursuit_follow_path(
                    path,
                    velocity=100.0,
                    lookahead=100.0,
                    tolerance=20.0,
                    blocking=False,
                    max_angular_rad_s=1.0,
                )
                print("[FSM] MOVING")
                state = "MOVING"

        elif state == "MOVING":
            show_moving_leds(robot)
            
            if robot.get_button(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                print("[FSM] IDLE")
                state = "IDLE"

            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
                robot.stop()
                print("[FSM] IDLE")
                state = "IDLE"

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
