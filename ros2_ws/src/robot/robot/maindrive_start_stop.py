"""
maindrive+start+stop.py — Full drive run with traffic light start and stop sign detection
==========================================================================================
- Waits for GREEN traffic light (or BTN_1) to start driving
- Drives the sectioned route with pure pursuit and APF goal stretches
- Stops automatically when stop sign is detected
- BTN_2 cancels at any time

cp /home/projectroger/Project-NUEVO-TsaoG4/ros2_ws/src/robot/robot/maindrive_start_stop.py /home/projectroger/Project-NUEVO-TsaoG4/ros2_ws/src/robot/robot/main.py

./ros2_ws/docker/restart.sh

./ros2_ws/docker/enter_ros2.sh

ros2 launch vision vision_debug.launch.py

ros2 run robot robot

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
    Stepper,
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

HOLD_STEPPERS_ENABLED = True
HOLD_STEPPERS = (Stepper.STEPPER_1, Stepper.STEPPER_2)

TAG_ID = 14

GPS_POSITION_ALPHA              = 0.05
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.30
GPS_TANGENT_MIN_DISPLACEMENT_MM = 180.0

# ---------------------------------------------------------------------------
# Pure pursuit first section
# ---------------------------------------------------------------------------
PP1_PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 3850.0),
    (525.0, 3750.0),
    (320.0, 3000.0),
]

PP2_PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 2250.0),
    (-750.0, 2250.0),
    (-750.0, 2050.0),
]

PP1_PATH_CONTROL_POINTS = densify_polyline(PP1_PATH_CONTROL_POINTS, spacing=50.0)
PP2_PATH_CONTROL_POINTS = densify_polyline(PP2_PATH_CONTROL_POINTS, spacing=50.0)

# ---------------------------------------------------------------------------
# APF second section
# ---------------------------------------------------------------------------
APF_GOAL_MM = (0.0, 2200.0)

# ---------------------------------------------------------------------------
# Final pure pursuit section
# ---------------------------------------------------------------------------
FINAL_PP1_PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 530.0),
    (730.0, 450.0),
]

FINAL_PP2_PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 3000.0),
]

FINAL_PP1_PATH_CONTROL_POINTS = densify_polyline(
    FINAL_PP1_PATH_CONTROL_POINTS,
    spacing=50.0,
)
FINAL_PP2_PATH_CONTROL_POINTS = densify_polyline(
    FINAL_PP2_PATH_CONTROL_POINTS,
    spacing=50.0,
)

# ---------------------------------------------------------------------------
# Final APF wall-avoidance stretch
# ---------------------------------------------------------------------------
FINAL_APF_GOAL_MM = (0.0, 1000.0)

# ---------------------------------------------------------------------------
# Post-final pure pursuit section
# ---------------------------------------------------------------------------
POST_FINAL_PP_PATH_CONTROL_POINTS = [
    (0.0, 0.0),
    (0.0, 1000.0),
    (-100.0, 1050.0),
]

POST_FINAL_PP_PATH_CONTROL_POINTS = densify_polyline(
    POST_FINAL_PP_PATH_CONTROL_POINTS,
    spacing=50.0,
)

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
# LAPF obstacle avoidance tuning
# ---------------------------------------------------------------------------
LEASH_LENGTH_MM = 400.0
REPULSION_RANGE_MM = 300.0
TARGET_SPEED_MM_S = 200.0
REPULSION_GAIN = 550.0
ATTRACTION_GAIN = 1.0
FORCE_EMA_ALPHA = 0.35
INFLATION_MARGIN_MM = 150.0
LEASH_HALF_ANGLE_DEG = 25.0

# ---------------------------------------------------------------------------
# Mission sections
# ---------------------------------------------------------------------------
MISSION_SECTIONS = [
    ("PP1", "pure_pursuit", PP1_PATH_CONTROL_POINTS),
    ("PP2", "pure_pursuit", PP2_PATH_CONTROL_POINTS),
    ("APF", "lapf_goal", APF_GOAL_MM),
    ("FINAL_PP1", "pure_pursuit", FINAL_PP1_PATH_CONTROL_POINTS),
    ("FINAL_PP2", "pure_pursuit", FINAL_PP2_PATH_CONTROL_POINTS),
    ("FINAL_APF", "lapf_goal", FINAL_APF_GOAL_MM),
    ("POST_FINAL_PP", "pure_pursuit", POST_FINAL_PP_PATH_CONTROL_POINTS),
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

    if HOLD_STEPPERS_ENABLED:
        for stepper in HOLD_STEPPERS:
            robot.step_enable(stepper)
        print("[stepper] holding torque enabled for pan/tilt steppers")

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


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()


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


def print_status(robot: Robot, section_name: str) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    status = f"  section={section_name} odom=({ox:6.0f},{oy:6.0f}) theta={otheta:5.1f}deg"

    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        status += f" fused=({fx:6.0f},{fy:6.0f}) theta_fused={ftheta:5.1f}deg"

    virtual_target = robot.get_virtual_target()
    if virtual_target is not None:
        status += f" vt=({virtual_target[0]:6.0f},{virtual_target[1]:6.0f})"

    if ENABLE_LIDAR:
        status += f" tracked={len(robot.get_obstacle_tracks())}"

    print(status)


def start_section(robot: Robot, section_index: int):
    name, mode, target = MISSION_SECTIONS[section_index]
    reset_mission_pose(robot)

    if mode == "pure_pursuit":
        print(f"[FSM] MOVING — {name} pure pursuit, {len(target)} waypoints")
        return robot.purepursuit_follow_path(
            waypoints=target,
            velocity=VELOCITY_MM_S,
            lookahead=LOOKAHEAD_MM,
            tolerance=TOLERANCE_MM,
            advance_radius=ADVANCE_RADIUS_MM,
            max_angular_rad_s=MAX_ANGULAR_RAD_S,
            blocking=False,
        )

    if mode == "lapf_goal":
        cfg = resolve_lapf_config()
        print(f"[FSM] MOVING — {name} APF goal {target}")
        return robot.lapf_to_goal(
            target[0],
            target[1],
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

    raise ValueError(f"unknown mission section mode: {mode}")


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state         = "INIT"
    drive_handle  = None
    section_index = 0
    last_status   = 0.0

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
                f"[CFG] sections={len(MISSION_SECTIONS)} "
                f"vel={VELOCITY_MM_S:.0f} mm/s "
                f"lookahead={LOOKAHEAD_MM:.0f} mm"
            )
            if ENABLE_LIDAR:
                print(
                    f"[CFG] lidar mount=({LIDAR_MOUNT_X_MM:.0f},{LIDAR_MOUNT_Y_MM:.0f}) mm "
                    f"filter={LIDAR_RANGE_MIN_MM:.0f}-{LIDAR_RANGE_MAX_MM:.0f} mm"
                )
            state = "IDLE"

        elif state == "IDLE":
            if GreenGo(robot) or robot.was_button_pressed(Button.BTN_1):
                show_moving_leds(robot)
                section_index = 0
                drive_handle = start_section(robot, section_index)
                last_status  = now
                state = "MOVING"

        elif state == "MOVING":
            section_name = MISSION_SECTIONS[section_index][0]

            if robot.was_button_pressed(Button.BTN_2):
                cancel_motion(robot, drive_handle)
                show_idle_leds(robot)
                print("[FSM] DONE — cancelled by BTN_2")
                state = "DONE"

            elif StopSignNear(robot):
                cancel_motion(robot, drive_handle)
                show_idle_leds(robot)
                print("[FSM] DONE — stop sign detected")
                state = "DONE"

            else:
                if now - last_status >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot, section_name)
                    last_status = now

                if drive_handle is not None and drive_handle.is_finished():
                    print(f"[FSM] complete — {section_name}")
                    print_status(robot, section_name)
                    section_index += 1
                    if section_index >= len(MISSION_SECTIONS):
                        robot.stop()
                        show_idle_leds(robot)
                        print("[FSM] DONE — mission complete")
                        state = "DONE"
                    else:
                        drive_handle = start_section(robot, section_index)
                        last_status = now

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
