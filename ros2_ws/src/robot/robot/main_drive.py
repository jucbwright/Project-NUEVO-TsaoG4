"""
Boustrophedon path sweep with APF obstacle avoidance and lidar/GPS fusion.

Press BTN_1 to start the path, BTN_2 to cancel.
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

ENABLE_LIDAR = True
ENABLE_GPS   = True

TAG_ID = 14

GPS_POSITION_ALPHA              = 0.05
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

# ---------------------------------------------------------------------------
# Path configuration
# ---------------------------------------------------------------------------
tile = 610.0  # mm (standard tile length)
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
    (tile*1,    tile*0),

    # Turn 2 — down from (1,0) to (2,0)
    (tile*1.33, tile*0),
    (tile*1.66, tile*0),
    (tile*2,    tile*0),

    # Straight 3 — right along row 2
    (tile*2,    tile*1),
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
    (tile*3,    tile*0),

    # Turn 4 — down from (3,0) to (4,0)
    (tile*3.33, tile*0),
    (tile*3.66, tile*0),
    (tile*4,    tile*0),
]

LOOKAHEAD_MM           = 120.0
ADVANCE_RADIUS_MM      = 80.0
TOLERANCE_MM           = 25.0
VELOCITY_MM_S          = 150.0
MAX_ANGULAR_RAD_S      = 1.0
APF_REPULSION_RANGE_MM = 300.0
PATH_DENSITY_MM        = 60.0

WAYPOINTS = densify_polyline(PATH_CONTROL_POINTS, PATH_DENSITY_MM)

STATUS_PRINT_INTERVAL_S = 0.5


def _make_obstacle_provider(robot: Robot):
    """Return a callback that converts world-frame obstacle tracks to robot-frame mm for APF."""
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


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


def print_status(robot: Robot) -> None:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_odometry_pose()
        label = "odom "

    virtual_target  = robot.get_virtual_target()
    obstacle_tracks = robot.get_obstacle_tracks()

    if virtual_target is None:
        vt_summary = " vt=(none)"
    else:
        vt_summary = f" vt=({virtual_target[0]:6.0f}, {virtual_target[1]:6.0f}) mm"

    if obstacle_tracks:
        nearest_boundary_mm = min(
            max(
                0.0,
                ((float(track["x"]) - x) ** 2 + (float(track["y"]) - y) ** 2) ** 0.5
                - float(track["radius"]),
            )
            for track in obstacle_tracks
        )
        track_summary = f" tracked={len(obstacle_tracks)} nearest_track={nearest_boundary_mm:.0f} mm"
    else:
        track_summary = " tracked=0"

    print(
        f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°"
        f"{vt_summary}{track_summary}"
    )


def start_path(robot: Robot):
    return robot.apf_follow_path(
        waypoints=WAYPOINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_range=APF_REPULSION_RANGE_MM,
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
            print(f"[CFG] waypoints={len(WAYPOINTS)} velocity={VELOCITY_MM_S:.0f} mm/s "
                  f"lookahead={LOOKAHEAD_MM:.0f} mm tolerance={TOLERANCE_MM:.0f} mm "
                  f"advance_radius={ADVANCE_RADIUS_MM:.0f} mm "
                  f"repulsion={APF_REPULSION_RANGE_MM:.0f} mm")
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
                print(
                    f"[CFG] tracker ttl={robot.OBSTACLE_TRACK_TTL_S:.1f}s "
                    f"max_tracks={robot.OBSTACLE_TRACK_MAX_TRACKS}"
                )
            if ENABLE_GPS:
                print(
                    f"[CFG] gps tag_id={TAG_ID} tag_body=({TAG_BODY_OFFSET_X_MM:.0f}, "
                    f"{TAG_BODY_OFFSET_Y_MM:.0f}) mm"
                )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                drive_handle = start_path(robot)
                last_status_print_at = now
                print(f"[FSM] MOVING — {len(WAYPOINTS)} waypoints")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, drive_handle)
                drive_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — path cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if drive_handle is not None and drive_handle.is_finished():
                    print("[FSM] DONE — goal complete")
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
