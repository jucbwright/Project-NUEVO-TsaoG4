"""
lapf_to_goal.py — leashed APF goal seeking with lidar
=====================================================
Single-goal obstacle avoidance using the new `lapf_to_goal()` API.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/lapf_to_goal.py main.py
    ros2 run robot robot

Press BTN_1 to start the run. BTN_2 cancels the active motion.

WHAT THIS TEACHES
-----------------
1. Optional lidar and GPS setup for local planning
2. Non-blocking `lapf_to_goal()` inside the standard FSM loop
3. How to inspect the moving virtual target while the rover runs
4. How the current default LAPF tuning behaves on one goal
"""

from __future__ import annotations

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

# Shared drive-base and lidar/GPS hardware calibration lives in
# robot/hardware_map.py. If you need to change wheel geometry, wheel motors,
# lidar mount, lidar self-filtering, or GPS tag body offset values, edit
# ros2_ws/src/robot/robot/hardware_map.py.
ENABLE_LIDAR = True
ENABLE_GPS = True

# IMPORTANT: update TAG_ID to match your robot when GPS is enabled.
TAG_ID = 14
# ---------------------------------------------------------------------------
# GPS tuning (only used when ENABLE_GPS = True)
#
# GPS_POSITION_ALPHA     — how strongly each GPS fix pulls the fused position.
#                          0.05 = smooth/slow, 0.10 = default, 0.30 = aggressive
#
# ENABLE_GPS_TANGENT_HEADING — derive heading from GPS trajectory direction.
#                          False = pure odometry heading (default).
#
# GPS_TANGENT_ALPHA      — how strongly GPS tangent corrects odometry heading.
#                          0.05 = gentle, 0.15 = default, 0.30 = aggressive
#
# GPS_TANGENT_MIN_DISPLACEMENT_MM — travel required before accepting a new
#                          heading sample. 100 = responsive, 200 = default,
#                          400 = noise-robust (for jittery GPS)
#
# To tune: watch θ_odom vs θ_fused in the status output while running.
# ---------------------------------------------------------------------------

GPS_POSITION_ALPHA           = 0.10
ENABLE_GPS_TANGENT_HEADING   = True
GPS_TANGENT_ALPHA            = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0


# ---------------------------------------------------------------------------
# Pure pursuit configuration
# ---------------------------------------------------------------------------
tile = 610.0 # mm (standard tile length))
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

LOOKAHEAD_MM       = 120.0
TOLERANCE_MM       = 25.0
GOAL_MM = (610.0, 610.0*5)
VELOCITY_MM_S = 150.0
TOLERANCE_MM = 50.0
MAX_ANGULAR_RAD_S = 1.0

# Edit these directly while tuning LAPF behavior.
LEASH_LENGTH_MM = 400.0 # This is for a front wheel drive; make it ~50 mm for a rear wheel drive
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

STATUS_PRINT_INTERVAL_S = 0.5


def resolve_lapf_config() -> dict[str, float]:
    return {
        "leash_length_mm": float(LEASH_LENGTH_MM),
        "repulsion_range_mm": float(REPULSION_RANGE_MM),
        "target_speed_mm_s": float(TARGET_SPEED_MM_S),
        "repulsion_gain": float(REPULSION_GAIN),
        "attraction_gain": float(ATTRACTION_GAIN),
        "force_ema_alpha": float(FORCE_EMA_ALPHA),
        "inflation_margin_mm": float(INFLATION_MARGIN_MM),
        "leash_half_angle_deg": float(LEASH_HALF_ANGLE_DEG),
    }


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

    virtual_target = robot.get_virtual_target()
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


def start_goal(robot: Robot):
    cfg = resolve_lapf_config()
    return robot.lapf_to_goal(
        GOAL_MM[0],
        GOAL_MM[1],
        velocity=VELOCITY_MM_S,
        tolerance=TOLERANCE_MM,
        leash_length_mm=cfg["leash_length_mm"],
        repulsion_range_mm=cfg["repulsion_range_mm"],
        target_speed_mm_s=cfg["target_speed_mm_s"],
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        repulsion_gain=cfg["repulsion_gain"],
        attraction_gain=cfg["attraction_gain"],
        force_ema_alpha=cfg["force_ema_alpha"],
        inflation_margin_mm=cfg["inflation_margin_mm"],
        leash_half_angle_deg=cfg["leash_half_angle_deg"],
        blocking=False,
    )


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    motion_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            lapf_cfg = resolve_lapf_config()
            print("[FSM] IDLE — press BTN_1 to start LAPF goal, BTN_2 to cancel")
            print(f"[CFG] goal={GOAL_MM} velocity={VELOCITY_MM_S:.0f} mm/s tolerance={TOLERANCE_MM:.0f} mm")
            print(
                f"[CFG] LAPF: leash={lapf_cfg['leash_length_mm']:.0f} mm "
                f"half_angle={lapf_cfg['leash_half_angle_deg']:.0f}° "
                f"target_speed={lapf_cfg['target_speed_mm_s']:.0f} mm/s "
                f"repulsion_range={lapf_cfg['repulsion_range_mm']:.0f} mm "
                f"repulsion_gain={lapf_cfg['repulsion_gain']:.0f} "
                f"attraction_gain={lapf_cfg['attraction_gain']:.2f} "
                f"force_ema_alpha={lapf_cfg['force_ema_alpha']:.2f} "
                f"inflation={lapf_cfg['inflation_margin_mm']:.0f} mm"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f}, {LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"theta={LIDAR_MOUNT_THETA_DEG:.1f}° filter={LIDAR_RANGE_MIN_MM:.0f}-"
                    f"{LIDAR_RANGE_MAX_MM:.0f} mm fov={LIDAR_FOV_DEG}"
                )
                print(
                    f"[CFG] tracker ttl={robot.OBSTACLE_TRACK_TTL_S:.1f}s "
                    f"max_tracks={robot.OBSTACLE_TRACK_MAX_TRACKS} "
                    f"planner_tracks={robot.LAPF_MAX_PLANNER_TRACKS}"
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
                motion_handle = start_goal(robot)
                last_status_print_at = now
                print("[FSM] MOVING — LAPF goal started")
                state = "MOVING"

        elif state == "MOVING":
            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, motion_handle)
                motion_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — LAPF goal cancelled")
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
                if motion_handle is not None and motion_handle.is_finished():
                    print("[FSM] DONE — goal complete")
                    print_status(robot)
                    motion_handle = None
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
