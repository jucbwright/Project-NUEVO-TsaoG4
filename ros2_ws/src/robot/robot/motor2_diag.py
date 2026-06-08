"""
motor2_diag.py — Motor 2 (left wheel) oscillation diagnostic.

Swap this in as main.py to run:
    cp motor2_diag.py main.py
    ros2 run robot robot

What it does (fully automated — no button presses needed until prompted):
  Phase 1 — Print velocity-loop PID gains for M1 vs M2 (compare).
  Phase 2 — Run M2 in raw PWM mode (no PID) for 5 s; measure velocity noise.
             If still oscillates → hardware fault. If smooth → PID issue.
  Phase 3 — Run M2 in velocity mode for 5 s; measure velocity noise.
  Phase 4 — If PID issue detected: press BTN_1 to apply fix, BTN_2 to skip.
  Phase 5 — Re-run velocity test after fix for confirmation.

Press BTN_2 at any point to abort.
"""

from __future__ import annotations

import math
import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DCPidLoop,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    POSITION_UNIT,
)
from robot.robot import FirmwareState, Robot

# ── Which motor to test ────────────────────────────────────────────────────────
TEST_MOTOR   = Motor.DC_M2      # left wheel — the one that jitters
REF_MOTOR    = Motor.DC_M1      # right wheel — reference

# ── Test parameters ────────────────────────────────────────────────────────────
PWM_LEVEL          = 80         # raw PWM during bypass test (0-255)
VEL_COMMAND        = 80.0       # mm/s during velocity test
TEST_DURATION_S    = 5.0        # how long each phase runs
SAMPLE_INTERVAL_S  = 0.1        # how often we sample dc_state
# A motor is "oscillating" if velocity std-dev > this fraction of |mean_vel|
OSCILLATION_THRESH = 0.40       # 40% — generous; clear oscillation is >> this

# ── PID fix to apply if oscillation is PID-driven ─────────────────────────────
# We halve Kp and raise Kd by 50 %.  Ki is left alone.
KP_SCALE = 0.50
KD_SCALE = 1.50


# ──────────────────────────────────────────────────────────────────────────────

def _banner(msg: str) -> None:
    bar = "─" * 60
    print(f"\n{bar}\n  {msg}\n{bar}")


def _sample_velocity(robot: Robot, duration_s: float) -> list[float]:
    """Collect motor-2 velocity readings over duration_s seconds."""
    samples: list[float] = []
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        state = robot.get_dc_state()
        if state is not None:
            # motors list is 0-indexed; motor 2 is index 1
            m = state.motors[TEST_MOTOR - 1]
            samples.append(float(m.velocity))
        time.sleep(SAMPLE_INTERVAL_S)
    return samples


def _stats(samples: list[float]) -> tuple[float, float]:
    """Return (mean, std_dev) of samples."""
    if not samples:
        return 0.0, 0.0
    n    = len(samples)
    mean = sum(samples) / n
    var  = sum((v - mean) ** 2 for v in samples) / n
    return mean, math.sqrt(var)


def _is_oscillating(samples: list[float]) -> bool:
    mean, std = _stats(samples)
    if abs(mean) < 1.0:
        # motor barely moving — can't judge by ratio; use absolute std
        return std > 10.0
    return (std / abs(mean)) > OSCILLATION_THRESH


def _print_pid(label: str, pid) -> None:
    if pid is None:
        print(f"  {label}: <no response — firmware may not have replied yet>")
    else:
        print(
            f"  {label}: kp={pid.kp:.4f}  ki={pid.ki:.4f}  kd={pid.kd:.4f}"
            f"  max_out={pid.max_output:.1f}  max_int={pid.max_integral:.1f}"
        )


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state          = "INIT"
    pid_m1         = None
    pid_m2         = None
    pwm_samples    = []
    vel_samples    = []
    vel2_samples   = []   # post-fix confirmation
    pid_fix_applied = False

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    # Timers / counters used across ticks
    phase_start: float = 0.0

    while True:

        # ── Abort anytime with BTN_2 ──────────────────────────────────────────
        if robot.was_button_pressed(Button.BTN_2) and state not in ("INIT", "DONE", "HW_FAULT"):
            robot.disable_motor(TEST_MOTOR)
            robot.set_led(LED.RED, 200)
            print("\n[DIAG] Aborted by BTN_2.")
            state = "DONE"

        # ─────────────────────────────────────────────────────────────────────
        if state == "INIT":
            fw = robot.get_state()
            if fw in (FirmwareState.ESTOP, FirmwareState.ERROR):
                robot.reset_estop()
            robot.set_state(FirmwareState.RUNNING)
            robot.set_led(LED.ORANGE, 200)
            _banner("Motor 2 Oscillation Diagnostic")
            print("  BTN_2 aborts at any time.\n")
            state = "READ_PID"

        # ── Phase 1: read PID gains ───────────────────────────────────────────
        elif state == "READ_PID":
            _banner("Phase 1 — Reading PID gains (velocity loop)")
            robot.request_pid(REF_MOTOR,  DCPidLoop.VELOCITY)
            robot.request_pid(TEST_MOTOR, DCPidLoop.VELOCITY)
            phase_start = time.monotonic()
            state = "WAIT_PID"

        elif state == "WAIT_PID":
            pid_m1 = robot.get_pid(REF_MOTOR,  DCPidLoop.VELOCITY)
            pid_m2 = robot.get_pid(TEST_MOTOR, DCPidLoop.VELOCITY)
            if pid_m1 is not None and pid_m2 is not None:
                _print_pid(f"Motor {REF_MOTOR}  (right / reference)", pid_m1)
                _print_pid(f"Motor {TEST_MOTOR} (left  / UNDER TEST )", pid_m2)
                if pid_m1 is not None and pid_m2 is not None:
                    if pid_m2.kp > pid_m1.kp * 1.2:
                        print(f"\n  [!] M2 Kp ({pid_m2.kp:.4f}) is significantly higher than"
                              f" M1 Kp ({pid_m1.kp:.4f}) — likely cause of oscillation.")
                    else:
                        print("\n  [ok] Kp values are similar between motors.")
                print("\n  Starting PWM bypass test in 2 s …")
                time.sleep(2.0)
                state = "PWM_START"
            elif time.monotonic() - phase_start > 3.0:
                print("  [warn] No PID response from firmware after 3 s — continuing anyway.")
                state = "PWM_START"

        # ── Phase 2: PWM bypass test ──────────────────────────────────────────
        elif state == "PWM_START":
            _banner(f"Phase 2 — PWM bypass test (no PID) for {TEST_DURATION_S:.0f} s")
            print(f"  Commanding motor {TEST_MOTOR} at fixed PWM={PWM_LEVEL}.")
            print("  Watch the motor. Does it still oscillate?\n")
            robot.enable_motor(TEST_MOTOR, DCMotorMode.PWM)
            time.sleep(0.05)
            robot.set_motor_pwm(TEST_MOTOR, PWM_LEVEL)
            pwm_samples = []
            phase_start = time.monotonic()
            state = "PWM_RUNNING"

        elif state == "PWM_RUNNING":
            elapsed = time.monotonic() - phase_start
            dc = robot.get_dc_state()
            if dc is not None:
                m = dc.motors[TEST_MOTOR - 1]
                pwm_samples.append(float(m.velocity))
                if len(pwm_samples) % 10 == 0:
                    mean, std = _stats(pwm_samples)
                    print(f"  t={elapsed:4.1f}s  vel={m.velocity:6.1f}  pwm={m.pwm_output:4d}"
                          f"  std_so_far={std:5.1f}")
            if elapsed >= TEST_DURATION_S:
                robot.set_motor_pwm(TEST_MOTOR, 0)
                robot.disable_motor(TEST_MOTOR)
                time.sleep(0.3)
                state = "PWM_RESULT"

        elif state == "PWM_RESULT":
            _banner("Phase 2 — PWM result")
            mean, std = _stats(pwm_samples)
            osc = _is_oscillating(pwm_samples)
            print(f"  Velocity mean={mean:.1f}  std={std:.1f}  oscillating={osc}")
            if osc:
                print(
                    "\n  [RESULT] Motor 2 oscillates even in raw PWM mode.\n"
                    "  This is a HARDWARE problem — PID tuning will not fix it.\n"
                    "  Check:\n"
                    "    1. Motor 2 wiring / connector (loose wire?)\n"
                    "    2. H-bridge channel 2 on the PCB\n"
                    "    3. Encoder connector for motor 2 (swap with M1 to test)\n"
                )
                robot.set_led(LED.RED, 200)
                state = "HW_FAULT"
            else:
                print(
                    "\n  [RESULT] Motor 2 ran smoothly in PWM mode.\n"
                    "  The oscillation is driven by the velocity PID loop.\n"
                    "  Proceeding to velocity-mode test …"
                )
                time.sleep(2.0)
                state = "VEL_START"

        # ── Phase 3: velocity-mode baseline ──────────────────────────────────
        elif state == "VEL_START":
            _banner(f"Phase 3 — Velocity mode test for {TEST_DURATION_S:.0f} s")
            print(f"  Commanding motor {TEST_MOTOR} at {VEL_COMMAND:.0f} mm/s.\n")
            robot.enable_motor(TEST_MOTOR, DCMotorMode.VELOCITY)
            time.sleep(0.05)
            robot.set_motor_velocity(TEST_MOTOR, VEL_COMMAND)
            vel_samples = []
            phase_start = time.monotonic()
            state = "VEL_RUNNING"

        elif state == "VEL_RUNNING":
            elapsed = time.monotonic() - phase_start
            dc = robot.get_dc_state()
            if dc is not None:
                m = dc.motors[TEST_MOTOR - 1]
                vel_samples.append(float(m.velocity))
                if len(vel_samples) % 10 == 0:
                    mean, std = _stats(vel_samples)
                    print(f"  t={elapsed:4.1f}s  vel={m.velocity:6.1f}"
                          f"  target={m.target_vel:6.1f}  pwm={m.pwm_output:4d}"
                          f"  std={std:5.1f}")
            if elapsed >= TEST_DURATION_S:
                robot.set_motor_velocity(TEST_MOTOR, 0.0)
                robot.disable_motor(TEST_MOTOR)
                time.sleep(0.3)
                state = "VEL_RESULT"

        elif state == "VEL_RESULT":
            _banner("Phase 3 — Velocity mode result")
            mean, std = _stats(vel_samples)
            osc = _is_oscillating(vel_samples)
            print(f"  Velocity mean={mean:.1f}  std={std:.1f}  oscillating={osc}")
            if osc:
                print("\n  [RESULT] Velocity PID is oscillating (confirmed).")
                if pid_m2 is not None:
                    new_kp = pid_m2.kp * KP_SCALE
                    new_kd = pid_m2.kd * KD_SCALE
                    print(
                        f"\n  Proposed fix:\n"
                        f"    kp: {pid_m2.kp:.4f} → {new_kp:.4f}  (×{KP_SCALE})\n"
                        f"    ki: {pid_m2.ki:.4f} → {pid_m2.ki:.4f}  (unchanged)\n"
                        f"    kd: {pid_m2.kd:.4f} → {new_kd:.4f}  (×{KD_SCALE})\n"
                        f"\n  Press BTN_1 to apply fix and re-test."
                        f"\n  Press BTN_2 to skip (or abort)."
                    )
                    robot.set_led(LED.ORANGE, 80)
                    state = "AWAIT_FIX"
                else:
                    print(
                        "\n  [warn] PID gains were not retrieved from firmware, so\n"
                        "  the auto-fix cannot be applied.\n"
                        "  Manually call robot.set_pid_gains() with lower kp.\n"
                    )
                    state = "DONE"
            else:
                print("\n  [ok] Velocity mode is NOT oscillating this run.")
                print("  The jitter may be intermittent. Try running main.py again\n"
                      "  and check if ENABLE_LIDAR/ENABLE_GPS are causing it.")
                state = "DONE"

        # ── Phase 4: apply fix ────────────────────────────────────────────────
        elif state == "AWAIT_FIX":
            if robot.was_button_pressed(Button.BTN_1):
                state = "APPLY_FIX"

        elif state == "APPLY_FIX":
            _banner("Phase 4 — Applying PID fix")
            new_kp = pid_m2.kp * KP_SCALE
            new_ki = pid_m2.ki
            new_kd = pid_m2.kd * KD_SCALE
            robot.set_pid_gains(
                TEST_MOTOR, DCPidLoop.VELOCITY,
                kp=new_kp, ki=new_ki, kd=new_kd,
                max_output=pid_m2.max_output,
                max_integral=pid_m2.max_integral,
            )
            print(f"  Applied: kp={new_kp:.4f}  ki={new_ki:.4f}  kd={new_kd:.4f}")
            pid_fix_applied = True
            print(f"\n  Re-testing for {TEST_DURATION_S:.0f} s …")
            time.sleep(1.0)
            state = "VEL2_START"

        # ── Phase 5: post-fix confirmation ────────────────────────────────────
        elif state == "VEL2_START":
            _banner(f"Phase 5 — Post-fix velocity test for {TEST_DURATION_S:.0f} s")
            robot.enable_motor(TEST_MOTOR, DCMotorMode.VELOCITY)
            time.sleep(0.05)
            robot.set_motor_velocity(TEST_MOTOR, VEL_COMMAND)
            vel2_samples = []
            phase_start = time.monotonic()
            state = "VEL2_RUNNING"

        elif state == "VEL2_RUNNING":
            elapsed = time.monotonic() - phase_start
            dc = robot.get_dc_state()
            if dc is not None:
                m = dc.motors[TEST_MOTOR - 1]
                vel2_samples.append(float(m.velocity))
                if len(vel2_samples) % 10 == 0:
                    mean, std = _stats(vel2_samples)
                    print(f"  t={elapsed:4.1f}s  vel={m.velocity:6.1f}"
                          f"  std={std:5.1f}")
            if elapsed >= TEST_DURATION_S:
                robot.set_motor_velocity(TEST_MOTOR, 0.0)
                robot.disable_motor(TEST_MOTOR)
                time.sleep(0.3)
                state = "VEL2_RESULT"

        elif state == "VEL2_RESULT":
            _banner("Phase 5 — Post-fix result")
            mean, std = _stats(vel2_samples)
            osc = _is_oscillating(vel2_samples)
            print(f"  Velocity mean={mean:.1f}  std={std:.1f}  oscillating={osc}")
            if not osc:
                print(
                    "\n  [SUCCESS] Motor 2 is no longer oscillating.\n"
                    "\n  IMPORTANT — these PID gains are only in RAM.\n"
                    "  To make them permanent, add this to configure_robot() in main.py:\n"
                )
                new_kp = pid_m2.kp * KP_SCALE
                new_ki = pid_m2.ki
                new_kd = pid_m2.kd * KD_SCALE
                print(
                    f"    from robot.hardware_map import Motor, DCPidLoop\n"
                    f"    robot.set_pid_gains(\n"
                    f"        Motor.DC_M2, DCPidLoop.VELOCITY,\n"
                    f"        kp={new_kp:.4f}, ki={new_ki:.4f}, kd={new_kd:.4f},\n"
                    f"        max_output={pid_m2.max_output:.1f},\n"
                    f"        max_integral={pid_m2.max_integral:.1f},\n"
                    f"    )\n"
                )
                robot.set_led(LED.GREEN, 200)
            else:
                print(
                    "\n  [PARTIAL] Still oscillating after halving Kp.\n"
                    "  Try halving again, or check encoder wiring.\n"
                )
                robot.set_led(LED.RED, 80)
            state = "DONE"

        # ── Terminal states ───────────────────────────────────────────────────
        elif state == "HW_FAULT":
            pass  # sit and wait; user reads the output

        elif state == "DONE":
            pass  # sit and wait; user reads the output

        # ── Tick rate ─────────────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
