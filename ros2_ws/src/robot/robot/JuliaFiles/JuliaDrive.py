"""
maindrive+start+stop.py — Full drive run with traffic light start and stop sign detection
==========================================================================================
- Waits for GREEN traffic light (or BTN_1) to start driving
- Drives 4-lane boustrophedon matching the competition course:

    Leg 1  (Pure Pursuit)  — Lane 0, north 6 tiles → Sec. 1 Checkpoint
    Top crossover          — 2x 90° right turns, east 1 tile to Lane 1
    Leg 2  (Pure Pursuit)  — Lane 1, south 5 tiles → Sec. 2 Checkpoint
    Bottom crossover       — 2x 90° left turns, east 1 tile to Lane 2
    Leg 3  (APF + LiDAR)   — Lane 2, north through obstacle field;
                             LiDAR wall-balance past barrier into Lane 3
    Leg 4  (Pure Pursuit)  — Lane 3, south: slow → speed up over bumps
                             → stop sign → Finish / Manipulation region

- Stops automatically when stop sign detected (armed near finish only)
- BTN_2 cancels at any time

Course geometry (top-down):
  X axis → East  (lane steps: 0, tile, 2*tile, 3*tile)
  Y axis → North (0 = start/finish row, 6*tile = top checkpoint row)

  Start:          (0,       0      )
  Sec. 1 Chkpt:   (0,       6*tile )  top of Lane 0
  Top crossover:  x: 0 → tile,   y: 6*tile
  Sec. 2 Chkpt:   (tile,    0      )  bottom of Lane 1 — also Sec.2 label on map
  Bottom cross:   x: tile → 2*tile, y: 0
  Obstacle field: Lane 2 (x≈2*tile), y = tile..5*tile  (cones + ramp)
  Sec. 3 Chkpt:   (2*tile,  6*tile )  top of Lane 2 / bottom crossover to Lane 3
  Lane 3 entry:   (3*tile,  6*tile )
  Speed bumps:    Lane 3 (x≈3*tile), scattered ~y = 1*tile..4*tile
  Stop sign:      (3*tile,  ~0.5*tile)
  Finish:         (3*tile,  0      )
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
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
# Course geometry
# ---------------------------------------------------------------------------
TILE_MM = 650.0
tile    = TILE_MM

# Lane centre X positions
LANE_0_X = tile * 0   # Leg 1 — pure pursuit, heading north
LANE_1_X = tile * 1   # Leg 2 — pure pursuit, heading south
LANE_2_X = tile * 2   # Leg 3 — APF + LiDAR obstacle avoidance, heading north
LANE_3_X = tile * 3   # Leg 4 — LiDAR wall-balance + speed bumps, heading south

BOTTOM_Y  = tile * 0
TOP_Y     = tile * 6

# Crossover arc overshoot: go slightly past the corner so turns are smooth
# and the rover doesn't cut the inner wall.
CROSS_OVERSHOOT = tile * 0.18

# ---------------------------------------------------------------------------
# Path control points
# ---------------------------------------------------------------------------

# ── Leg 1: Pure Pursuit — Lane 0, driving NORTH ─────────────────────────────
LEG1_POINTS = [
    (LANE_0_X, BOTTOM_Y),
    (LANE_0_X, tile * 1),
    (LANE_0_X, tile * 2),
    (LANE_0_X, tile * 3),
    (LANE_0_X, tile * 4),
    (LANE_0_X, tile * 5),
    (LANE_0_X, TOP_Y),       # Sec. 1 Checkpoint
]

# ── Top crossover: 2x 90° right → Lane 0 to Lane 1, heading south ───────────
# First 90° right: north→east along the top wall
# Second 90° right: east→south into Lane 1
TOP_CROSS_POINTS = [
    (LANE_0_X,                          TOP_Y),
    (LANE_0_X,                          TOP_Y + CROSS_OVERSHOOT),  # overshoot north
    (LANE_0_X + tile * 0.5,             TOP_Y + CROSS_OVERSHOOT),  # east mid-arc
    (LANE_1_X,                          TOP_Y + CROSS_OVERSHOOT),  # reach Lane 1 X
    (LANE_1_X,                          TOP_Y),                    # drop back to top wall
]

# ── Leg 2: Pure Pursuit — Lane 1, driving SOUTH ─────────────────────────────
LEG2_POINTS = [
    (LANE_1_X, TOP_Y),
    (LANE_1_X, tile * 5),
    (LANE_1_X, tile * 4),
    (LANE_1_X, tile * 3),
    (LANE_1_X, tile * 2),
    (LANE_1_X, tile * 1),
    (LANE_1_X, BOTTOM_Y),    # Sec. 2 Checkpoint
]

# ── Bottom crossover: 2x 90° left → Lane 1 to Lane 2, heading north ─────────
# First 90° left: south→east along the bottom wall
# Second 90° left: east→north into Lane 2
BOTTOM_CROSS_POINTS = [
    (LANE_1_X,                          BOTTOM_Y),
    (LANE_1_X,                          BOTTOM_Y - CROSS_OVERSHOOT),  # overshoot south
    (LANE_1_X + tile * 0.5,             BOTTOM_Y - CROSS_OVERSHOOT),  # east mid-arc
    (LANE_2_X,                          BOTTOM_Y - CROSS_OVERSHOOT),  # reach Lane 2 X
    (LANE_2_X,                          BOTTOM_Y),                    # come back to bottom wall
]

# ── Leg 3: APF + LiDAR — Lane 2, driving NORTH through obstacle field ────────
# APF handles cone avoidance. LiDAR wall-balance keeps the rover centred
# in the lane while navigating past the ramp/barrier structure mid-lane.
LEG3_POINTS = [
    (LANE_2_X, BOTTOM_Y),
    (LANE_2_X, tile * 1),
    (LANE_2_X, tile * 2),
    (LANE_2_X, tile * 3),    # mid obstacle field (ramp zone)
    (LANE_2_X, tile * 4),
    (LANE_2_X, tile * 5),
    (LANE_2_X, TOP_Y),       # Sec. 3 Checkpoint — top of Lane 2
]

# ── Top crossover 2: right into Lane 3, heading south ───────────────────────
# Same arc geometry as the first top crossover.
TOP_CROSS_2_POINTS = [
    (LANE_2_X,                          TOP_Y),
    (LANE_2_X,                          TOP_Y + CROSS_OVERSHOOT),
    (LANE_2_X + tile * 0.5,             TOP_Y + CROSS_OVERSHOOT),
    (LANE_3_X,                          TOP_Y + CROSS_OVERSHOOT),
    (LANE_3_X,                          TOP_Y),
]

# ── Leg 4: LiDAR wall-balance — Lane 3, driving SOUTH over speed bumps ───────
# Speed bumps: rover slows before each bump waypoint, then accelerates after.
# The speed profile is handled in start_segment via velocity overrides.
LEG4_POINTS = [
    (LANE_3_X, TOP_Y),
    (LANE_3_X, tile * 5),
    (LANE_3_X, tile * 4),    # speed bump zone start
    (LANE_3_X, tile * 3),    # speed bump mid
    (LANE_3_X, tile * 2),    # speed bump zone end
    (LANE_3_X, tile * 1),
    (LANE_3_X, tile * 0.5),  # stop sign arm zone — stop sign near here
    (LANE_3_X, BOTTOM_Y),    # Finish / Manipulation region
]

# ---------------------------------------------------------------------------
# Densified waypoints per segment
# ---------------------------------------------------------------------------
WAYPOINT_SPACING_MM = 80.0

# Segment definition: (name, mode, control_points)
# mode = "pure_pursuit" | "apf_lidar" | "lidar_wall_bumps"
_RAW_SEGMENTS = [
    ("Leg 1 — lane 0 north (pure pursuit)",           "pure_pursuit",     LEG1_POINTS),
    ("Top crossover 1 — lane 0→1 (pure pursuit)",     "pure_pursuit",     TOP_CROSS_POINTS),
    ("Leg 2 — lane 1 south (pure pursuit)",           "pure_pursuit",     LEG2_POINTS),
    ("Bottom crossover — lane 1→2 (pure pursuit)",    "pure_pursuit",     BOTTOM_CROSS_POINTS),
    ("Leg 3 — lane 2 north (APF + LiDAR obstacles)",  "apf_lidar",        LEG3_POINTS),
    ("Top crossover 2 — lane 2→3 (pure pursuit)",     "pure_pursuit",     TOP_CROSS_2_POINTS),
    ("Leg 4 — lane 3 south (LiDAR wall + bumps)",     "lidar_wall_bumps", LEG4_POINTS),
]

PATH_SEGMENTS = [
    (name, mode, densify_polyline(pts, spacing=WAYPOINT_SPACING_MM))
    for name, mode, pts in _RAW_SEGMENTS
]

TOTAL_SEGMENT_WAYPOINTS = sum(len(wps) for _, _, wps in PATH_SEGMENTS)

# ---------------------------------------------------------------------------
# Navigation parameters
# ---------------------------------------------------------------------------
VELOCITY_MM_S         = 5.0
LOOKAHEAD_MM          = 140.0
TOLERANCE_MM          = 35.0
ADVANCE_RADIUS_MM     = 60.0
MAX_ANGULAR_RAD_S     = 0.6

# APF — obstacle lane (Leg 3)
APF_REPULSION_RANGE_MM = 60.0
APF_REPULSION_GAIN     = 35.0
ROBOT_FRONT_MM         = 120.0
ROBOT_REAR_MM          = 360.0
ROBOT_HALF_WIDTH_MM    = 200.0

# Speed bump profile (Leg 4)
# Rover slows to BUMP_APPROACH_VEL when within BUMP_SLOW_RADIUS of a bump
# y-coordinate, then returns to VELOCITY_MM_S after clearing BUMP_CLEAR_Y.
BUMP_SLOW_RADIUS_MM   = 200.0
BUMP_APPROACH_VEL     = 40.0   # mm/s — slow crawl over the bump
BUMP_Y_POSITIONS      = [tile * 4, tile * 3, tile * 2]  # approx bump Y coords

# LiDAR wall-balance (Legs 3 and 4)
LIDAR_CENTRE_KP       = 0.2
LIDAR_CENTRE_DEADBAND = 30.0    # mm — ignore small lateral drift

STATUS_PRINT_INTERVAL_S = 0.5

# Pause after Leg 1 (segment 0), before the top crossover's two 90° turns.
LEG1_PAUSE_S = 1.5

# Settle time after pre-enabling both drive motors (see configure_robot), so
# M1 and M2 are both already in VELOCITY mode before the first motion command —
# avoids M1 lagging M2 on the very first move of a segment.
DRIVE_ENABLE_SETTLE_S = 0.2

# ---------------------------------------------------------------------------
# Spatial guards
# ---------------------------------------------------------------------------
# APF repulsion only active in Lane 2 bounding box
APF_LANE_X_MIN = LANE_2_X - tile * 0.45
APF_LANE_X_MAX = LANE_2_X + tile * 0.45
APF_LANE_Y_MIN = BOTTOM_Y
APF_LANE_Y_MAX = TOP_Y

# Stop-sign arm: only active in Lane 3 near the bottom
STOP_SIGN_ARM_X_MIN = LANE_3_X - tile * 0.45
STOP_SIGN_ARM_Y_MAX = tile * 1.0

# ---------------------------------------------------------------------------
# Section markers for status printing
# ---------------------------------------------------------------------------
SECTION_MARKERS = [
    ("start",              LANE_0_X, BOTTOM_Y),
    ("leg1 mid",           LANE_0_X, tile * 3),
    ("sec1 checkpoint",    LANE_0_X, TOP_Y),
    ("top cross 1",        LANE_1_X, TOP_Y),
    ("leg2 mid",           LANE_1_X, tile * 3),
    ("sec2 checkpoint",    LANE_1_X, BOTTOM_Y),
    ("bottom cross",       LANE_2_X, BOTTOM_Y),
    ("leg3 obstacle mid",  LANE_2_X, tile * 3),
    ("sec3 checkpoint",    LANE_2_X, TOP_Y),
    ("top cross 2",        LANE_3_X, TOP_Y),
    ("leg4 bump zone",     LANE_3_X, tile * 3),
    ("stop sign zone",     LANE_3_X, tile * 0.5),
    ("finish",             LANE_3_X, BOTTOM_Y),
]

# ---------------------------------------------------------------------------
# Runtime state
# ---------------------------------------------------------------------------
ACTIVE_SEGMENT_MODE: str = "pure_pursuit"


def _set_mode(mode: str) -> None:
    global ACTIVE_SEGMENT_MODE
    ACTIVE_SEGMENT_MODE = mode


def _apf_active(robot: Robot) -> bool:
    if ACTIVE_SEGMENT_MODE not in ("apf_lidar",):
        return False
    x, y, _ = robot.get_pose()
    return APF_LANE_X_MIN <= x <= APF_LANE_X_MAX and APF_LANE_Y_MIN <= y <= APF_LANE_Y_MAX


def _make_obstacle_provider(robot: Robot):
    def _provider():
        if not _apf_active(robot):
            return []
        tracks = robot.get_obstacle_tracks()
        if not tracks:
            return []
        if robot.has_fused_pose():
            rx, ry, rtheta_deg = robot.get_fused_pose()
        else:
            rx, ry, rtheta_deg = robot.get_odometry_pose()
        rtheta = math.radians(rtheta_deg)
        cos_t, sin_t = math.cos(-rtheta), math.sin(-rtheta)
        result = []
        for t in tracks:
            dx = float(t['x']) - rx
            dy = float(t['y']) - ry
            result.append((cos_t * dx - sin_t * dy, sin_t * dx + cos_t * dy))
        return result
    return _provider


def _bump_velocity(robot: Robot) -> float:
    """Return reduced speed if rover is close to a speed bump, else nominal."""
    _, y, _ = robot.get_pose()
    for bump_y in BUMP_Y_POSITIONS:
        if abs(y - bump_y) < BUMP_SLOW_RADIUS_MM:
            return BUMP_APPROACH_VEL
    return VELOCITY_MM_S


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

    # Pre-enable both drive motors in VELOCITY mode together and zero their
    # targets immediately. enable() does not clear a stale non-zero target
    # left over from a previous run, so without this one motor can lurch to
    # its old setpoint as soon as it's enabled while the other sits at zero.
    robot.enable_motor(LEFT_WHEEL_MOTOR, DCMotorMode.VELOCITY)
    robot.enable_motor(RIGHT_WHEEL_MOTOR, DCMotorMode.VELOCITY)
    robot.stop()
    time.sleep(DRIVE_ENABLE_SETTLE_S)

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


def get_nav_pose(robot: Robot) -> tuple[float, float, float]:
    return robot.get_pose()


def estimate_section(x: float, y: float) -> str:
    label, tx, ty = min(SECTION_MARKERS, key=lambda m: (x - m[1])**2 + (y - m[2])**2)
    return label


def stop_sign_armed(robot: Robot) -> bool:
    x, y, _ = get_nav_pose(robot)
    return x >= STOP_SIGN_ARM_X_MIN and y <= STOP_SIGN_ARM_Y_MAX


def start_segment(robot: Robot, seg_idx: int):
    name, mode, waypoints = PATH_SEGMENTS[seg_idx]
    _set_mode(mode)
    print(
        f"[FSM] segment {seg_idx + 1}/{len(PATH_SEGMENTS)} — {name} "
        f"mode={mode} waypoints={len(waypoints)}"
    )

    if mode == "apf_lidar":
        # APF for cone avoidance + LiDAR wall-balance to stay centred in lane
        robot.enable_lidar_lane_centering(
            kp=LIDAR_CENTRE_KP,
            deadband_mm=LIDAR_CENTRE_DEADBAND,
        )
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

    if mode == "lidar_wall_bumps":
        # LiDAR wall-balance for lane centering; velocity managed per-tick
        # in the FSM loop to slow/speed for speed bumps
        robot.enable_lidar_lane_centering(
            kp=LIDAR_CENTRE_KP,
            deadband_mm=LIDAR_CENTRE_DEADBAND,
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

    # Default: pure_pursuit
    return robot.purepursuit_follow_path(
        waypoints=waypoints,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_MM,
        tolerance=TOLERANCE_MM,
        advance_radius=ADVANCE_RADIUS_MM,
        max_angular_rad_s=MAX_ANGULAR_RAD_S,
        blocking=False,
    )


def end_segment(robot: Robot, mode: str) -> None:
    if mode in ("apf_lidar", "lidar_wall_bumps"):
        robot.disable_lidar_lane_centering()


def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    nx, ny, ntheta = get_nav_pose(robot)
    tracks = robot.get_obstacle_tracks()
    if tracks:
        nearest = min(
            max(0.0, ((float(t["x"]) - nx)**2 + (float(t["y"]) - ny)**2)**0.5 - float(t["radius"]))
            for t in tracks
        )
        track_str = f"  tracked={len(tracks)} nearest={nearest:.0f} mm"
    else:
        track_str = "  tracked=0"
    src = "fused" if robot.has_fused_pose() else "odom"
    sec = estimate_section(nx, ny)
    print(
        f"  nav[{src}]=({nx:6.0f},{ny:6.0f}) θ={ntheta:5.1f}° "
        f"odom=({ox:6.0f},{oy:6.0f}) θ={otheta:5.1f}° "
        f"sec={sec} mode={ACTIVE_SEGMENT_MODE} "
        f"apf={'on' if _apf_active(robot) else 'off'} "
        f"stop_arm={'yes' if stop_sign_armed(robot) else 'no'}"
        f"{track_str}"
    )


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------
def run(robot: Robot) -> None:
    configure_robot(robot)

    state         = "INIT"
    drive_handle  = None
    seg_idx       = 0
    last_status   = 0.0
    last_bump_vel = VELOCITY_MM_S
    pause_until   = 0.0

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
                f"[CFG] segments={len(PATH_SEGMENTS)} "
                f"total_wps={TOTAL_SEGMENT_WAYPOINTS} "
                f"vel={VELOCITY_MM_S:.0f} mm/s "
                f"lookahead={LOOKAHEAD_MM:.0f} mm "
                f"apf_range={APF_REPULSION_RANGE_MM:.0f} mm"
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
                seg_idx      = 0
                drive_handle = start_segment(robot, seg_idx)
                last_status  = now
                print(f"[FSM] MOVING — {TOTAL_SEGMENT_WAYPOINTS} total waypoints")
                state = "MOVING"

        elif state == "MOVING":
            # BTN_2: cancel
            if robot.was_button_pressed(Button.BTN_2):
                if drive_handle:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                _, cur_mode, _ = PATH_SEGMENTS[seg_idx]
                end_segment(robot, cur_mode)
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — cancelled by BTN_2")
                state = "DONE"

            # Stop-sign detection (armed only near finish)
            elif stop_sign_armed(robot) and StopSignNear(robot):
                if drive_handle:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                _, cur_mode, _ = PATH_SEGMENTS[seg_idx]
                end_segment(robot, cur_mode)
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — stop sign detected")
                state = "DONE"

            else:
                # Speed bump velocity management (Leg 4 only)
                _, cur_mode, _ = PATH_SEGMENTS[seg_idx]
                if cur_mode == "lidar_wall_bumps" and drive_handle is not None:
                    target_vel = _bump_velocity(robot)
                    if target_vel != last_bump_vel:
                        drive_handle.set_velocity(target_vel)
                        last_bump_vel = target_vel
                        action = "slowing for bump" if target_vel < VELOCITY_MM_S else "resuming speed"
                        print(f"[bumps] {action} → {target_vel:.0f} mm/s")

                # Status print
                if now - last_status >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status = now

                # Advance to next segment
                if drive_handle is not None and drive_handle.is_finished():
                    finished_idx = seg_idx
                    _, finished_mode, _ = PATH_SEGMENTS[seg_idx]
                    end_segment(robot, finished_mode)
                    seg_idx += 1
                    last_bump_vel = VELOCITY_MM_S
                    drive_handle = None

                    if seg_idx >= len(PATH_SEGMENTS):
                        print("[FSM] DONE — all segments complete")
                        print_status(robot)
                        robot.stop()
                        show_idle_leds(robot)
                        state = "DONE"
                    elif finished_idx == 0:
                        # Pause after Leg 1, before the top crossover's two 90° turns.
                        robot.stop()
                        print(f"[FSM] PAUSED — Leg 1 complete, holding {LEG1_PAUSE_S:.1f}s before crossover")
                        pause_until = now + LEG1_PAUSE_S
                        state = "PAUSED"
                    else:
                        drive_handle = start_segment(robot, seg_idx)

        elif state == "PAUSED":
            # BTN_2: cancel during the pause
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — cancelled by BTN_2")
                state = "DONE"
            elif now >= pause_until:
                drive_handle = start_segment(robot, seg_idx)
                last_status = now
                print("[FSM] MOVING — resuming after Leg 1 pause")
                state = "MOVING"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()