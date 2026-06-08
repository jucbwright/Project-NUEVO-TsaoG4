from __future__ import annotations

from robot.maindrive_start_stop import run as run_drive
from robot.JuliaFiles.ZombieSweep import run as run_sweep
from robot.robot import Robot


def run(robot: Robot) -> None:
    run_drive(robot)
    run_sweep(robot)
