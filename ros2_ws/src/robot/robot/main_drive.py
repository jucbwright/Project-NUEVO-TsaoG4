"""
Boustrophedon path sweep with APF obstacle avoidance and LiDAR.

Combines:
  - pure_pursuit.py  : path structure and waypoints
  - obstacle_avoidance_apf.py : LiDAR setup and APF obstacle avoidance tuning

Press BTN_1 to start, BTN_2 to cancel.
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline

# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------
ENABLE_LIDAR = True
ENABLE_GPS   = True

TAG_ID = 14

GPS_POSITION_ALPHA              = 0.05
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

# ---------------------------------------------------------------------------
# Path — boustrophedon lawnmower (from pure_pursuit.py blueprint)
# ---------------------------------------------------------------------------
tile = 650.0
# Straight line test — 3 tiles forward
# INITIAL_THETA_DEG=180 means robot faces south (-Y), so path goes south
PATH_CONTROL_POINTS = [
    # Straight 1 — right along row 0
    (tile*0,    tile*0),
    (tile*0,    tile*6),

    # Turn 1 — down from (0,6) to (1,6)
    (tile*0.33, tile*6),
    (tile*0.66, tile*6),
    (tile*1,    tile*6),

    # Straight 2 — left along row 1
    (tile*1,    tile*5),
    (tile*1,    tile*4),
    (tile*1,    tile*3),
    (tile*1,    tile*2),
    (tile*1,    tile*1),

    # Turn 2 — down from (1,1) to (2,1)
    (tile*1.33, tile*1),
    (tile*1.66, tile*1),
    (tile*2,    tile*1),

    # Straight 3
    (tile*2,    tile*2),
    (tile*2,    tile*3),
    (tile*2,    tile*4),
    (tile*2,    tile*5),
    (tile*2,    tile*6),

    # Turn 3 — down from (2,6) to (3,6)
    (tile*2.33, tile*6),
    (tile*2.66, tile*6),
    (tile*3,    tile*6),

    # Straight 4 — left along row 3
    (tile*3,    tile*5),
    (tile*3,    tile*4),
    (tile*3,    tile*3),
    (tile*3,    tile*2),
    (tile*3,    tile*1),

    # Turn 4 — down from (3,1) to (4,1)
    (tile*3.33, tile*1),
    (tile*3.66, tile*1),
    (tile*4,    tile*1),

    # Final straight
    (tile*4,    tile*0),
]

WAYPOINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=60.0)

# ---------------------------------------------------------------------------
# Pure pursuit tuning (from pure_pursuit.py blueprint)
# ---------------------------------------------------------------------------
VELOCITY_MM_S     = 180.0
LOOKAHEAD_MM      = 250.0
TOLERANCE_MM      = 25.0
ADVANCE_RADIUS_MM = 80.0
MAX_ANGULAR_RAD_S = 1.5

# ---------------------------------------------------------------------------
# APF obstacle avoidance tuning (from obstacle_avoidance_apf.py blueprint)
# ---------------------------------------------------------------------------
APF_REPULSION_RANGE_MM = 150.0
APF_REPULSION_GAIN     = 200.0

STATUS_PRINT_INTERVAL_S = 0.5


def _make_obstacle_provider(robot: Robot):
    """Convert world-frame tracked obstacles to robot-frame mm for APF."""
    def _provider():
        tracks = robot.get_obstacle_tracks()
        if not tracks:
            return []
        if robot.has_fused_pose():
            rx, ry, rtheta = robot.get_fused_pose()
        else:
            rx, ry, rtheta = robot.get_odometry_pose()
        cos_t = math.cos(-rtheta)
        sin_t = math.sin(-rtheta)
        result = []
        for t in tracks:
            dx = float(t['x']) - rx
            dy = float(t['y']) - ry
            result.append((cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy))
        return result
    return _provider


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

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        robot.set_obstacle_provider(_make_obstacle_provider(robot))
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")
        if ENABLE_GPS_TANGENT_HEADING:
            robot.enable_gps_tangent_heading(
                alpha=GPS_TANGENT_ALPHA,
                min_displacement_mm=GPS_TANGENT_MIN_DISPLACEMENT_MM,
            )


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°"
        )
    else:
        obstacle_tracks = robot.get_obstacle_tracks()
        if obstacle_tracks:
            nearest = min(
                max(0.0,
                    ((float(t["x"]) - ox) ** 2 + (float(t["y"]) - oy) ** 2) ** 0.5
                    - float(t["radius"]))
                for t in obstacle_tracks
            )
            track_summary = f"  tracked={len(obstacle_tracks)} nearest={nearest:.0f} mm"
        else:
            track_summary = "  tracked=0"
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°{track_summary}")


def start_path(robot: Robot):
    return robot.apf_follow_path(
        waypoints=WAYPOINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_range=APF_REPULSION_RANGE_MM,
        repulsion_gain=APF_REPULSION_GAIN,
        blocking=False,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    last_status_print_at = 0.0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start path, BTN_2 to cancel")
            print(
                f"[CFG] waypoints={len(WAYPOINTS)} velocity={VELOCITY_MM_S:.0f} mm/s "
                f"lookahead={LOOKAHEAD_MM:.0f} mm tolerance={TOLERANCE_MM:.0f} mm "
                f"repulsion={APF_REPULSION_RANGE_MM:.0f} mm"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f},{LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"filter={LIDAR_RANGE_MIN_MM:.0f}-{LIDAR_RANGE_MAX_MM:.0f} mm"
                )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_moving_leds(robot)
                drive_handle = start_path(robot)
                last_status_print_at = now
                print(f"[FSM] MOVING — {len(WAYPOINTS)} waypoints")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — path cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] DONE — path complete")
                    print_status(robot)
                    drive_handle = None
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 to run again")
                    state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
