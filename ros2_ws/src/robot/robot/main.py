"""
main.py — Pure pursuit + stepper test (combined)
=================================================
BTN_1 — start pure pursuit path
BTN_2 — cancel pure pursuit (returns to IDLE)
BTN_3 — stepper single run: both steppers CW 200 steps once
BTN_4 — stepper loop test: both steppers alternate CW/CCW for 10 cycles

HOW TO RUN
----------
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
    StepMoveType,
    Stepper,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline  # noqa: F401 - optional helper for students


# ---------------------------------------------------------------------------
# Sensor toggles
# ---------------------------------------------------------------------------
ENABLE_LIDAR = True
ENABLE_GPS   = True

TAG_ID = 14  # IMPORTANT: set to the ArUco marker ID on your robot


# ---------------------------------------------------------------------------
# GPS tuning
# ---------------------------------------------------------------------------
GPS_POSITION_ALPHA              = 0.10
ENABLE_GPS_TANGENT_HEADING      = True
GPS_TANGENT_ALPHA               = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0


# ---------------------------------------------------------------------------
# Pure pursuit configuration
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

VELOCITY_MM_S      = 150.0
LOOKAHEAD_MM       = 120.0
TOLERANCE_MM       = 25.0
ADVANCE_RADIUS_MM  = 80.0
MAX_ANGULAR_RAD_S  = 1.5

STATUS_PRINT_INTERVAL_S = 0.5


# ---------------------------------------------------------------------------
# Stepper configuration — change these to match your PCB connectors
# ---------------------------------------------------------------------------
DRIVE_STEPPER_L = Stepper.STEPPER_2    # Left drive motor
DRIVE_STEPPER_R = Stepper.STEPPER_3    # Right drive motor

STEPS_PER_RUN   = 200                  # Steps per move
MAX_VELOCITY    = 800                  # Steps/sec
ACCELERATION    = 400                  # Steps/sec²
STEP_CYCLES     = 10                   # Number of loop cycles
PAUSE_BETWEEN_S = 1.0                  # Pause between cycles (seconds)
MOVE_TIMEOUT_S  = 10.0                 # Timeout per move (seconds)


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------
def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# Pure pursuit helpers
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


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")


def start_path(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=PATH_CONTROL_POINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


# ---------------------------------------------------------------------------
# Stepper helpers
# ---------------------------------------------------------------------------
def enable_drive_steppers(robot: Robot) -> None:
    robot.step_set_config(DRIVE_STEPPER_L, max_velocity=MAX_VELOCITY, acceleration=ACCELERATION)
    robot.step_set_config(DRIVE_STEPPER_R, max_velocity=MAX_VELOCITY, acceleration=ACCELERATION)
    robot.step_enable(DRIVE_STEPPER_L)
    robot.step_enable(DRIVE_STEPPER_R)


def disable_drive_steppers(robot: Robot) -> None:
    robot.step_disable(DRIVE_STEPPER_L)
    robot.step_disable(DRIVE_STEPPER_R)


def move_both(robot: Robot, steps: int) -> bool:
    ok_l = robot.step_move(
        DRIVE_STEPPER_L,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=MOVE_TIMEOUT_S,
    )
    ok_r = robot.step_move(
        DRIVE_STEPPER_R,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=MOVE_TIMEOUT_S,
    )
    return ok_l and ok_r


def run_single(robot: Robot) -> None:
    print("[SINGLE] Starting — Steppers CW 200 steps")
    enable_drive_steppers(robot)
    ok = move_both(robot, STEPS_PER_RUN)
    disable_drive_steppers(robot)
    if ok:
        print("[SINGLE] Done")
    else:
        print("[SINGLE] Warning — one or both steppers timed out")


def run_loop(robot: Robot) -> None:
    print(f"[LOOP] Starting — {STEP_CYCLES} cycles, alternating CW/CCW")
    enable_drive_steppers(robot)
    for ii in range(STEP_CYCLES):
        steps = STEPS_PER_RUN if ii % 2 == 0 else -STEPS_PER_RUN
        direction = "CW" if steps > 0 else "CCW"
        print(f"[LOOP] Cycle {ii + 1}/{STEP_CYCLES} — {direction}")
        ok = move_both(robot, steps)
        if not ok:
            print(f"[LOOP] Warning — timeout on cycle {ii + 1}, stopping early")
            break
        time.sleep(PAUSE_BETWEEN_S)
    disable_drive_steppers(robot)
    print("[LOOP] Done")


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — BTN_1=pursuit start | BTN_2=pursuit cancel | BTN_3=stepper single | BTN_4=stepper loop")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                reset_mission_pose(robot)
                show_running_leds(robot)
                print(f"[FSM] PURSUIT — {len(PATH_CONTROL_POINTS)} waypoints")
                drive_handle = start_path(robot)
                last_status_print_at = now
                state = "PURSUIT"

            elif robot.was_button_pressed(Button.BTN_3):
                show_running_leds(robot)
                print("[FSM] STEPPER_SINGLE")
                state = "STEPPER_SINGLE"

            elif robot.was_button_pressed(Button.BTN_4):
                show_running_leds(robot)
                print("[FSM] STEPPER_LOOP")
                state = "STEPPER_LOOP"

        elif state == "PURSUIT":
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

        elif state == "STEPPER_SINGLE":
            run_single(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — BTN_1=pursuit start | BTN_2=pursuit cancel | BTN_3=stepper single | BTN_4=stepper loop")
            state = "IDLE"

        elif state == "STEPPER_LOOP":
            run_loop(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — BTN_1=pursuit start | BTN_2=pursuit cancel | BTN_3=stepper single | BTN_4=stepper loop")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
