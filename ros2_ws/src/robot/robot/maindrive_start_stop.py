"""
maindrive+start+stop.py — Full drive run with traffic light start and stop sign detection
==========================================================================================
- Waits for GREEN traffic light (or BTN_1) to start driving
- Drives boustrophedon path with pure pursuit
- Stops automatically when stop sign is detected (armed near finish only)
- BTN_2 cancels at any time
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline
from robot.start import LookForGreen, GreenGo
from robot.stop import LookForStop, StopSignNear

# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------
ENABLE_GPS = False

TAG_ID = 14

GPS_POSITION_ALPHA              = 0.05
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.30
GPS_TANGENT_MIN_DISPLACEMENT_MM = 180.0

# ---------------------------------------------------------------------------
# Course geometry
# ---------------------------------------------------------------------------
TILE_MM = 650.0
tile    = TILE_MM

# ---------------------------------------------------------------------------
# Path control points
# ---------------------------------------------------------------------------
PATH_CONTROL_POINTS = [
    # Straight 1
    (tile*0,    tile*0),
    (tile*0,    tile*6),

    # Turn 1
    (tile*0.33, tile*6),
    (tile*0.66, tile*6),
    (tile*1,    tile*6),

    # Straight 2
    (tile*1,    tile*5),
    (tile*1,    tile*4),
    (tile*1,    tile*3),
    (tile*1,    tile*2),
    (tile*1,    tile*1),

    # Turn 2
    (tile*1.33, tile*1),
    (tile*1.66, tile*1),
    (tile*2,    tile*1),

    # Straight 3
    (tile*2,    tile*2),
    (tile*2,    tile*3),
    (tile*2,    tile*4),
    (tile*2,    tile*5),
    (tile*2,    tile*6),

    # Turn 3
    (tile*2.33, tile*6),
    (tile*2.66, tile*6),
    (tile*3,    tile*6),

    # Straight 4
    (tile*3,    tile*5),
    (tile*3,    tile*4),
    (tile*3,    tile*3),
    (tile*3,    tile*2),
    (tile*3,    tile*1),

    # Turn 4
    (tile*3.33, tile*1),
    (tile*3.66, tile*1),
    (tile*4,    tile*1),

    # Final straight
    (tile*4,    tile*0),
]

WAYPOINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=80.0)

# ---------------------------------------------------------------------------
# Navigation parameters
# ---------------------------------------------------------------------------
VELOCITY_MM_S     = 10.0
LOOKAHEAD_MM      = 160.0
TOLERANCE_MM      = 25.0
ADVANCE_RADIUS_MM = 90.0
MAX_ANGULAR_RAD_S = 1.5

STATUS_PRINT_INTERVAL_S = 0.5

# ---------------------------------------------------------------------------
# Stop sign spatial guard
# ---------------------------------------------------------------------------
STOP_SIGN_ARM_X_MIN = tile * 3.5
STOP_SIGN_ARM_Y_MAX = tile * 1.5

# ---------------------------------------------------------------------------
# Section markers
# ---------------------------------------------------------------------------
SECTION_MARKERS = [
    ("lane 1 up",             tile * 0, tile * 6),
    ("top crossover 1->2",    tile * 1, tile * 6),
    ("lane 2 down",           tile * 1, tile * 1),
    ("bottom crossover 2->3", tile * 2, tile * 1),
    ("lane 3 up",             tile * 2, tile * 6),
    ("top crossover 3->4",    tile * 3, tile * 6),
    ("lane 4 down",           tile * 3, tile * 1),
    ("bottom crossover 4->5", tile * 4, tile * 1),
    ("finish lane",           tile * 4, tile * 0),
]


# ---------------------------------------------------------------------------
# Robot configuration
# ---------------------------------------------------------------------------
def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")
        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )

    LookForGreen(robot)
    LookForStop(robot)
    print("[sensor] vision enabled — watching for traffic light and stop sign")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def stop_sign_armed(robot: Robot) -> bool:
    x, y, _ = robot.get_pose()
    return x >= STOP_SIGN_ARM_X_MIN and y <= STOP_SIGN_ARM_Y_MAX


def estimate_section(x: float, y: float) -> str:
    label, tx, ty = min(SECTION_MARKERS, key=lambda m: (x - m[1])**2 + (y - m[2])**2)
    return label


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    sec = estimate_section(ox, oy)
    print(
        f"  odom=({ox:6.0f},{oy:6.0f}) θ={otheta:5.1f}°  "
        f"sec={sec}  "
        f"stop_arm={'yes' if stop_sign_armed(robot) else 'no'}"
    )


def start_path(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=WAYPOINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state        = "INIT"
    drive_handle = None
    last_status  = 0.0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while state != "DONE":
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — waiting for GREEN light (or press BTN_1)")
            print(
                f"[CFG] waypoints={len(WAYPOINTS)} "
                f"vel={VELOCITY_MM_S:.0f} mm/s "
                f"lookahead={LOOKAHEAD_MM:.0f} mm"
            )
            print(
                f"[CFG] stop-sign armed: x>={STOP_SIGN_ARM_X_MIN:.0f} mm "
                f"AND y<={STOP_SIGN_ARM_Y_MAX:.0f} mm"
            )
            state = "IDLE"

        elif state == "IDLE":
            if GreenGo(robot) or robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_moving_leds(robot)
                drive_handle = start_path(robot)
                last_status  = now
                print(f"[FSM] MOVING — {len(WAYPOINTS)} waypoints")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — cancelled by BTN_2")
                state = "DONE"

            elif stop_sign_armed(robot) and StopSignNear(robot):
                if drive_handle:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — stop sign detected")
                state = "DONE"

            else:
                if now - last_status >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status = now

                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] DONE — path complete")
                    print_status(robot)
                    robot.stop()
                    show_idle_leds(robot)
                    state = "DONE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()