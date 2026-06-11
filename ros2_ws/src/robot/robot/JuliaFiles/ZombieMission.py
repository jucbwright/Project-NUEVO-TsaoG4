"""
ZombieMission.py — drive, find stop sign, home camera, then zombie sweep
==========================================================================
Single self-contained mission script merging Transition.py and
ZombieSweep.py into one FSM:

  1. DRIVE_FORWARD  — drive forward DRIVE_TILES tiles (open loop)
  2. WATCH_STOP     — watch the camera for a stop sign in the lower-right
                       quadrant of the frame (mirrors ZombieSweep's R_DOWN
                       grid position)
  3. HOME_STEPPERS  — return the pan/tilt camera to its starting (0, 0)
                       position
  4. SWEEP          — run the pan/tilt grid scan, lighting all LEDs when a
                       zombie is centred at a grid position

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp JuliaFiles/ZombieMission.py main.py
    ros2 run robot robot

BTN_2 cancels the active phase at any time and stops the robot.
"""

from __future__ import annotations

import time
from typing import NamedTuple

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    INITIAL_THETA_DEG,
    LED,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    StepMoveType,
    Stepper,
    StepperMotionState,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot
from robot.util import TaskHandle, run_task


# ---------------------------------------------------------------------------
# Drive configuration
# ---------------------------------------------------------------------------
TILE_MM             = 650.0     # matches maindrive_start_stop.py grid spacing
DRIVE_TILES         = 4
DRIVE_DISTANCE_MM   = TILE_MM * DRIVE_TILES
DRIVE_VELOCITY_MM_S = 220.0
DRIVE_TOLERANCE_MM  = 25.0

DRIVE_ENABLE_SETTLE_S = 0.2

# ---------------------------------------------------------------------------
# Stop-sign watch configuration (lower-right quadrant of frame)
# ---------------------------------------------------------------------------
STOP_MIN_CONFIDENCE  = 0.50
VISION_STALE_SEC     = 3.0
WATCH_STOP_TIMEOUT_S = 10.0   # give up watching and home the camera anyway

# ---------------------------------------------------------------------------
# Pan/tilt stepper configuration (matches ZombieSweep.py)
# ---------------------------------------------------------------------------
PAN_STEPPER  = Stepper.STEPPER_1
TILT_STEPPER = Stepper.STEPPER_2

PAN_MAX_VEL  = 800    # steps/s
PAN_ACCEL    = 400    # steps/s^2

TILT_MAX_VEL = 300
TILT_ACCEL   = 150

AIM_TIMEOUT_S = 10.0   # max seconds per stepper move
DWELL_S       = 3.0    # seconds to hold each grid position

ZOMBIE_MIN_CONFIDENCE = 0.30
LED_BRIGHTNESS        = 255
DWELL_POLL_S          = 0.2

ALL_LEDS = (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE)

DEADZONE_PX = 30   # pixels either side of frame centre

PAN_STEPS_PER_PIXEL    = 0.15
PAN_MAX_STEPS_PER_MOVE = 15
PAN_COOLDOWN_SEC       = 0.6
PAN_MIN, PAN_MAX = -500, 500

TILT_STEPS_PER_PIXEL    = 0.50
TILT_MAX_STEPS_PER_MOVE = 5
TILT_COOLDOWN_SEC       = 1.50
TILT_MIN, TILT_MAX = -100, 100

CENTER_STILL_SEC = 1.0   # seconds a zombie must stay centred before "found"

PAN_L, PAN_C, PAN_R    = -200,   0,  200   # pan columns: left / center / right
TILT_U, TILT_C, TILT_D =  50,   0, -50    # tilt rows:   up   / center / down


class Target(NamedTuple):
    label: str
    pan:   int    # absolute steps from startup origin (PAN_STEPPER)
    tilt:  int    # absolute steps from startup origin (TILT_STEPPER)


SEQUENCE: list[Target] = [
    Target("HOME",   PAN_C, TILT_C),
    Target("L_DOWN", PAN_L, TILT_D),
    Target("L_CTR",  PAN_L, TILT_C),
    Target("L_UP",   PAN_L, TILT_U),
    Target("C_UP",   PAN_C, TILT_U),
    Target("C_CTR",  PAN_C, TILT_C),
    Target("C_DOWN", PAN_C, TILT_D),
    Target("R_DOWN", PAN_R, TILT_D),
    Target("R_CTR",  PAN_R, TILT_C),
    Target("R_UP",   PAN_R, TILT_U),
    Target("HOME",   PAN_C, TILT_C),
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

    robot.enable_motor(LEFT_WHEEL_MOTOR, DCMotorMode.VELOCITY)
    robot.enable_motor(RIGHT_WHEEL_MOTOR, DCMotorMode.VELOCITY)
    robot.stop()
    time.sleep(DRIVE_ENABLE_SETTLE_S)

    robot.enable_vision()

    robot.step_set_config(PAN_STEPPER,  max_velocity=PAN_MAX_VEL,  acceleration=PAN_ACCEL)
    robot.step_set_config(TILT_STEPPER, max_velocity=TILT_MAX_VEL, acceleration=TILT_ACCEL)
    robot.step_enable(PAN_STEPPER)
    robot.step_enable(TILT_STEPPER)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)


# ---------------------------------------------------------------------------
# Stop-sign helper (lower-right quadrant) — mirrors Transition.py
# ---------------------------------------------------------------------------

def _stop_sign_in_lower_right(robot: Robot) -> bool:
    """True if the highest-confidence stop-sign detection this tick is
    centred in the lower-right quadrant of the camera frame."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False

    frame_w, frame_h = robot.get_detection_image_size()
    if frame_w <= 0 or frame_h <= 0:
        return False

    best, best_conf = None, -1.0
    for det in robot.get_detections("stop sign"):
        conf = float(det["confidence"])
        if conf < STOP_MIN_CONFIDENCE:
            continue
        if conf > best_conf:
            best_conf = conf
            best = det

    if best is None:
        return False

    bbox = best["bbox"]
    cx = bbox["x"] + bbox["width"] / 2.0
    cy = bbox["y"] + bbox["height"] / 2.0
    return cx > frame_w / 2.0 and cy > frame_h / 2.0


# ---------------------------------------------------------------------------
# Zombie sweep helpers — mirror ZombieSweep.py
# ---------------------------------------------------------------------------

def _select_target_zombie(robot: Robot, frame_w: int, frame_h: int) -> dict | None:
    """Among all zombie detections, return the one nearest to the frame
    centre, or None if no zombie is detected."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None
    target_x, target_y = frame_w / 2.0, frame_h / 2.0
    best, best_dist = None, float("inf")
    for det in robot.get_detections("zombie"):
        if float(det["confidence"]) < ZOMBIE_MIN_CONFIDENCE:
            continue
        bbox = det["bbox"]
        cx = bbox["x"] + bbox["width"] / 2.0
        cy = bbox["y"] + bbox["height"] / 2.0
        dist = (cx - target_x) ** 2 + (cy - target_y) ** 2
        if dist < best_dist:
            best_dist = dist
            best = det
    return best


def _stepper_is_idle(robot: Robot, stepper: Stepper) -> bool:
    """Return True when the stepper is not actively moving. FAULT and
    unknown states are treated as idle so a stale firmware state never
    silently freezes the centering correction."""
    step_state = robot.get_step_state()
    if step_state is None:
        return True
    motion = step_state.steppers[int(stepper) - 1].motion_state
    return motion not in (
        int(StepperMotionState.ACCEL),
        int(StepperMotionState.CRUISE),
        int(StepperMotionState.DECEL),
    )


def _dim_all_leds(robot: Robot) -> None:
    for led in ALL_LEDS:
        robot.set_led(led, 0)


# ---------------------------------------------------------------------------
# LED status helpers
# ---------------------------------------------------------------------------

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 0)


def show_driving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def show_watching_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.RED, 0)


def show_triggered_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.RED, 200)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# Sweep sequence worker — runs in a background task while the FSM keeps
# ticking so BTN_2 can still cancel it.
# ---------------------------------------------------------------------------

def _sweep_worker(robot: Robot, pos: dict, task: TaskHandle) -> None:
    for target in SEQUENCE:
        if task.cancelled():
            return

        print(f"[SWEEP] -> {target.label}  pan={target.pan}  tilt={target.tilt}")
        if target.pan != pos["pan"]:
            ok = robot.step_move(
                PAN_STEPPER, steps=target.pan, move_type=StepMoveType.ABSOLUTE,
                blocking=True, timeout=AIM_TIMEOUT_S,
            )
            if not ok:
                print(f"[warn] pan timed out before reaching {target.label}")
            pos["pan"] = target.pan
        if target.tilt != pos["tilt"]:
            ok = robot.step_move(
                TILT_STEPPER, steps=target.tilt, move_type=StepMoveType.ABSOLUTE,
                blocking=True, timeout=AIM_TIMEOUT_S,
            )
            if not ok:
                print(f"[warn] tilt timed out before reaching {target.label}")
            pos["tilt"] = target.tilt

        print(f"[SWEEP] aimed at {target.label} — holding {DWELL_S}s")
        dwell_end = time.monotonic() + DWELL_S
        zombie_found = False
        still_since: float | None = None
        pan_next_allowed = 0.0
        tilt_next_allowed = 0.0

        while time.monotonic() < dwell_end:
            if task.cancelled():
                return
            now = time.monotonic()
            frame_w, frame_h = robot.get_detection_image_size()
            zombie = _select_target_zombie(robot, frame_w, frame_h)

            if zombie is None:
                still_since = None
                _dim_all_leds(robot)
                robot.set_led(LED.ORANGE, 200)
                time.sleep(DWELL_POLL_S)
                continue

            bbox = zombie["bbox"]
            zombie_cx = bbox["x"] + bbox["width"] / 2.0
            zombie_cy = bbox["y"] + bbox["height"] / 2.0
            pan_error  = zombie_cx - frame_w / 2.0 if frame_w > 0 else 0.0
            tilt_error = zombie_cy - frame_h / 2.0 if frame_h > 0 else 0.0
            centered = abs(pan_error) <= DEADZONE_PX and abs(tilt_error) <= DEADZONE_PX

            if centered:
                if still_since is None:
                    still_since = now
                if not zombie_found and (now - still_since) >= CENTER_STILL_SEC:
                    zombie_found = True
                    print(f"[ZOMBIE] Centered at {target.label}! Holding 2s for shot...")
                    for led in ALL_LEDS:
                        robot.set_led(led, LED_BRIGHTNESS)
            else:
                still_since = None

            if not zombie_found:
                robot.set_led(LED.GREEN, LED_BRIGHTNESS)
                robot.set_led(LED.ORANGE, 0)

                # --- Pan correction (horizontal) ---
                if (frame_w > 0 and now >= pan_next_allowed
                        and _stepper_is_idle(robot, PAN_STEPPER)
                        and abs(pan_error) > DEADZONE_PX):
                    steps = int(
                        max(-PAN_MAX_STEPS_PER_MOVE,
                            min(PAN_MAX_STEPS_PER_MOVE, -pan_error * PAN_STEPS_PER_PIXEL))
                    )
                    new_pos = max(PAN_MIN, min(PAN_MAX, pos["pan"] + steps))
                    steps   = new_pos - pos["pan"]
                    if steps != 0:
                        robot.step_move(PAN_STEPPER, steps=steps,
                                        move_type=StepMoveType.RELATIVE, blocking=False)
                        pos["pan"] = new_pos
                        pan_next_allowed = now + PAN_COOLDOWN_SEC

                # --- Tilt correction (vertical) ---
                if (frame_h > 0 and now >= tilt_next_allowed
                        and _stepper_is_idle(robot, TILT_STEPPER)
                        and abs(tilt_error) > DEADZONE_PX):
                    steps = int(
                        max(-TILT_MAX_STEPS_PER_MOVE,
                            min(TILT_MAX_STEPS_PER_MOVE, -tilt_error * TILT_STEPS_PER_PIXEL))
                    )
                    new_pos = max(TILT_MIN, min(TILT_MAX, pos["tilt"] + steps))
                    steps   = new_pos - pos["tilt"]
                    if steps != 0:
                        robot.step_move(TILT_STEPPER, steps=steps,
                                        move_type=StepMoveType.RELATIVE, blocking=False)
                        pos["tilt"] = new_pos
                        tilt_next_allowed = now + TILT_COOLDOWN_SEC

            time.sleep(DWELL_POLL_S)

        if zombie_found:
            for led in ALL_LEDS:
                robot.set_led(led, LED_BRIGHTNESS)
            shoot_end = time.monotonic() + 2.0
            while time.monotonic() < shoot_end:
                if task.cancelled():
                    return
                time.sleep(DWELL_POLL_S)
            _dim_all_leds(robot)

        show_running_leds(robot)


# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state        = "INIT"
    drive_handle = None
    task_handle  = None
    pos          = {"pan": 0, "tilt": 0}   # tracks current stepper positions
    watch_until  = 0.0

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while state != "DONE":
        now = time.monotonic()

        if state == "INIT":
            current = robot.get_state()
            if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            reset_mission_pose(robot)

            show_driving_leds(robot)
            print(f"[FSM] DRIVE_FORWARD — {DRIVE_TILES} tiles ({DRIVE_DISTANCE_MM:.0f} mm)")
            drive_handle = robot.move_forward(
                DRIVE_DISTANCE_MM,
                velocity=DRIVE_VELOCITY_MM_S,
                tolerance=DRIVE_TOLERANCE_MM,
                blocking=False,
            )
            state = "DRIVE_FORWARD"

        elif state == "DRIVE_FORWARD":
            if robot.was_button_pressed(Button.BTN_2):
                drive_handle.cancel()
                drive_handle.wait(timeout=1.0)
                drive_handle = None
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — cancelled during DRIVE_FORWARD")
                state = "DONE"
            elif drive_handle is not None and drive_handle.is_finished():
                drive_handle = None
                robot.stop()
                watch_until = now + WATCH_STOP_TIMEOUT_S
                show_watching_leds(robot)
                print("[FSM] WATCH_STOP — watching for stop sign in lower-right quadrant")
                state = "WATCH_STOP"

        elif state == "WATCH_STOP":
            if robot.was_button_pressed(Button.BTN_2):
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] DONE — cancelled during WATCH_STOP")
                state = "DONE"
            elif _stop_sign_in_lower_right(robot):
                show_triggered_leds(robot)
                print("[FSM] stop sign seen in lower-right quadrant — homing camera")
                state = "HOME_STEPPERS"
            elif now >= watch_until:
                print("[FSM] WATCH_STOP timed out — homing camera anyway")
                state = "HOME_STEPPERS"

        elif state == "HOME_STEPPERS":
            robot.step_move(PAN_STEPPER,  steps=0, move_type=StepMoveType.ABSOLUTE,
                             blocking=True, timeout=AIM_TIMEOUT_S)
            robot.step_move(TILT_STEPPER, steps=0, move_type=StepMoveType.ABSOLUTE,
                             blocking=True, timeout=AIM_TIMEOUT_S)
            pos["pan"] = pos["tilt"] = 0

            show_running_leds(robot)
            print("[FSM] SWEEP — starting zombie sweep sequence")

            def _worker(task: TaskHandle, pos=pos) -> None:
                _sweep_worker(robot, pos, task)

            task_handle = run_task(_worker, blocking=False)
            state = "SWEEP"

        elif state == "SWEEP":
            if robot.was_button_pressed(Button.BTN_2):
                task_handle.cancel()
                task_handle.wait(timeout=AIM_TIMEOUT_S + 1.0)
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] DONE — sweep cancelled")
                state = "DONE"
            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] DONE — sweep complete")
                state = "DONE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()