"""
maindrive+start+stop.py — Full drive run with traffic light start and stop sign detection
==========================================================================================
- Waits for GREEN traffic light (or BTN_1) to start driving
- Drives boustrophedon path with LiDAR APF obstacle avoidance
- Stops automatically when stop sign is detected
- BTN_2 cancels at any time

NOTE: To use this in robot_node.py, rename it to a valid Python module name
      (no '+' characters), e.g. main_drive_full.py
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
from robot.start import LookForGreen, GreenGo
from robot.stop import LookForStop, StopSignNear

# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------
ENABLE_LIDAR = True
ENABLE_GPS   = True

TAG_ID = 14

GPS_POSITION_ALPHA              = 0.05
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.30
GPS_TANGENT_MIN_DISPLACEMENT_MM = 180.0

# ---------------------------------------------------------------------------
# Path
# ---------------------------------------------------------------------------
TILE_MM = 650.0
tile = TILE_MM  # mm (course tile length)
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

    # Straight 3 — right along row 2 (obstacle avoidance handles cones later)
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

    # Final straight — left along row 4 to end at (4,0)
    (tile*4,    tile*0),
]

GOAL_MM = (TILE_MM, TILE_MM * 5)
WAYPOINT_SPACING_MM = 80.0
WAYPOINTS = densify_polyline(PATH_CONTROL_POINTS, spacing=WAYPOINT_SPACING_MM)

# Split the mission so only Straight 3 uses LiDAR/APF. All other sections are
# pure pursuit using the current navigation pose, which means odom/GPS fusion
# can influence pose but LiDAR cannot steer those sections.
PURE_PURSUIT_SEGMENT_1_POINTS = PATH_CONTROL_POINTS[:14]
APF_STRAIGHT_3_POINTS = [
    (tile * 2, tile * 2),
    (tile * 2, tile * 3),
    (tile * 2, tile * 4),
    (tile * 2, tile * 5),
    (tile * 2, tile * 6),
]
PURE_PURSUIT_SEGMENT_2_POINTS = [APF_STRAIGHT_3_POINTS[-1], *PATH_CONTROL_POINTS[18:]]

PATH_SEGMENTS = [
    ("pure pursuit before Straight 3", False, densify_polyline(PURE_PURSUIT_SEGMENT_1_POINTS, spacing=WAYPOINT_SPACING_MM)),
    ("APF obstacle lane Straight 3", True, densify_polyline(APF_STRAIGHT_3_POINTS, spacing=WAYPOINT_SPACING_MM)),
    ("pure pursuit after Straight 3", False, densify_polyline(PURE_PURSUIT_SEGMENT_2_POINTS, spacing=WAYPOINT_SPACING_MM)),
]
TOTAL_SEGMENT_WAYPOINTS = sum(len(points) for _name, _uses_apf, points in PATH_SEGMENTS)


VELOCITY_MM_S          = 220.0
LOOKAHEAD_MM           = 160.0
TOLERANCE_MM           = 25.0
ADVANCE_RADIUS_MM      = 90.0
MAX_ANGULAR_RAD_S      = 1.5
APF_REPULSION_RANGE_MM = 60.0
APF_REPULSION_GAIN     = 35.0
ROBOT_FRONT_MM         = 120.0
ROBOT_REAR_MM          = 360.0
ROBOT_HALF_WIDTH_MM    = 200.0
STATUS_PRINT_INTERVAL_S = 0.5
ACTIVE_SEGMENT_USES_APF = False

# Only use LiDAR repulsion in the obstacle lane. The early/top turns should be
# path-tracking only; otherwise stale or off-course tracks can make APF flip the
# avoidance direction and oscillate left/right at the turn.
APF_OBSTACLE_LANE_X_MIN_MM = tile * 1.5
APF_OBSTACLE_LANE_X_MAX_MM = tile * 2.5
APF_OBSTACLE_LANE_Y_MIN_MM = tile * 2.0
APF_OBSTACLE_LANE_Y_MAX_MM = tile * 6.0

# Stop-sign detections are only mission-ending near the finish lane. This
# prevents a visible sign, poster, or classifier glitch from cancelling the
# drive during the earlier straight sections. These values match the restored
# 650 mm tile waypoint frame above.
STOP_SIGN_ARM_X_MIN_MM = tile * 3.5
STOP_SIGN_ARM_Y_MAX_MM = tile * 1.5

SECTION_MARKERS = [
    ("lane 1 up", tile * 0, tile * 6),
    ("top crossover 1->2", tile * 1, tile * 6),
    ("lane 2 down", tile * 1, tile * 1),
    ("bottom crossover 2->3", tile * 2, tile * 1),
    ("lane 3 up", tile * 2, tile * 6),
    ("top crossover 3->4", tile * 3, tile * 6),
    ("lane 4 down", tile * 3, tile * 1),
    ("bottom crossover 4->5", tile * 4, tile * 1),
    ("finish lane", tile * 4, tile * 0),
]


def _apf_obstacles_enabled(robot: Robot) -> bool:
    x_mm, y_mm, _theta = robot.get_pose()
    return (
        ACTIVE_SEGMENT_USES_APF
        and APF_OBSTACLE_LANE_X_MIN_MM <= x_mm <= APF_OBSTACLE_LANE_X_MAX_MM
        and APF_OBSTACLE_LANE_Y_MIN_MM <= y_mm <= APF_OBSTACLE_LANE_Y_MAX_MM
    )


def _clear_apf_segment() -> None:
    global ACTIVE_SEGMENT_USES_APF
    ACTIVE_SEGMENT_USES_APF = False


def _make_obstacle_provider(robot: Robot):
    def _provider():
        if not _apf_obstacles_enabled(robot):
            return []
        tracks = robot.get_obstacle_tracks()
        if not tracks:
            return []
        if robot.has_fused_pose():
            rx, ry, rtheta_deg = robot.get_fused_pose()
        else:
            rx, ry, rtheta_deg = robot.get_odometry_pose()
        rtheta = math.radians(rtheta_deg)
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

    # Enable vision for traffic light and stop sign detection
    LookForGreen(robot)
    LookForStop(robot)
    print("[sensor] vision enabled — watching for traffic light and stop sign")


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


def get_nav_pose(robot: Robot) -> tuple[float, float, float]:
    return robot.get_pose()


def estimate_next_section(x_mm: float, y_mm: float) -> str:
    label, target_x, target_y = min(
        SECTION_MARKERS,
        key=lambda item: (x_mm - item[1]) ** 2 + (y_mm - item[2]) ** 2,
    )
    return label


def stop_sign_armed(robot: Robot) -> bool:
    x_mm, y_mm, _theta = get_nav_pose(robot)
    return x_mm >= STOP_SIGN_ARM_X_MIN_MM and y_mm <= STOP_SIGN_ARM_Y_MAX_MM


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    nx, ny, ntheta = get_nav_pose(robot)
    obstacle_tracks = robot.get_obstacle_tracks()
    if obstacle_tracks:
        nearest = min(
            max(0.0,
                ((float(t["x"]) - nx) ** 2 + (float(t["y"]) - ny) ** 2) ** 0.5
                - float(t["radius"]))
            for t in obstacle_tracks
        )
        track_summary = f"  tracked={len(obstacle_tracks)} nearest={nearest:.0f} mm"
    else:
        track_summary = "  tracked=0"
    pose_source = "fused" if robot.has_fused_pose() else "odom"
    section = estimate_next_section(nx, ny)
    print(
        f"  nav[{pose_source}]=({nx:6.0f}, {ny:6.0f}) mm theta={ntheta:5.1f} deg "
        f"odom=({ox:6.0f}, {oy:6.0f}) theta={otheta:5.1f} deg "
        f"section={section} apf={'yes' if _apf_obstacles_enabled(robot) else 'no'} stop_arm={'yes' if stop_sign_armed(robot) else 'no'}"
        f"{track_summary}"
    )


def start_segment(robot: Robot, segment_index: int):
    global ACTIVE_SEGMENT_USES_APF
    name, uses_apf, waypoints = PATH_SEGMENTS[segment_index]
    ACTIVE_SEGMENT_USES_APF = uses_apf
    print(
        f"[FSM] segment {segment_index + 1}/{len(PATH_SEGMENTS)} — {name} "
        f"waypoints={len(waypoints)}"
    )
    if uses_apf:
        return robot.apf_follow_path(
            waypoints=waypoints,
            velocity=VELOCITY_MM_S,
            lookahead=LOOKAHEAD_MM,
            tolerance=TOLERANCE_MM,
            advance_radius=ADVANCE_RADIUS_MM,
            max_angular_rad_s=MAX_ANGULAR_RAD_S,
            repulsion_range=APF_REPULSION_RANGE_MM,
            repulsion_gain=APF_REPULSION_GAIN,
            robot_front_mm=ROBOT_FRONT_MM,
            robot_rear_mm=ROBOT_REAR_MM,
            robot_half_width_mm=ROBOT_HALF_WIDTH_MM,
            blocking=False,
        )
    return robot.purepursuit_follow_path(
        waypoints=waypoints,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    segment_index = 0
    last_status_print_at = 0.0

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
                f"[CFG] segments={len(PATH_SEGMENTS)} waypoints={TOTAL_SEGMENT_WAYPOINTS} velocity={VELOCITY_MM_S:.0f} mm/s "
                f"lookahead={LOOKAHEAD_MM:.0f} mm repulsion={APF_REPULSION_RANGE_MM:.0f} mm"
            )
            print(
                f"[CFG] stop sign armed only after x>={STOP_SIGN_ARM_X_MIN_MM:.0f} mm "
                f"and y<={STOP_SIGN_ARM_Y_MAX_MM:.0f} mm"
            )
            state = "IDLE"

        elif state == "IDLE":
            # Start on green traffic light OR BTN_1 manual override
            if GreenGo(robot) or robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_moving_leds(robot)
                segment_index = 0
                drive_handle = start_segment(robot, segment_index)
                last_status_print_at = now
                print(f"[FSM] MOVING — {TOTAL_SEGMENT_WAYPOINTS} total waypoints")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                _clear_apf_segment()
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — path cancelled, proceeding to sweep")
                state = "DONE"

            elif stop_sign_armed(robot) and StopSignNear(robot):
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                _clear_apf_segment()
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — stop sign detected, proceeding to sweep")
                state = "DONE"

            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if drive_handle is not None and drive_handle.is_finished():
                    segment_index += 1
                    if segment_index < len(PATH_SEGMENTS):
                        drive_handle = start_segment(robot, segment_index)
                    else:
                        print("[FSM] DONE — path complete, proceeding to sweep")
                        print_status(robot)
                        drive_handle = None
                        _clear_apf_segment()
                        robot.stop()
                        show_idle_leds(robot)
                        state = "DONE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
