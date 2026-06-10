from __future__ import annotations

from robot.maindrive_start_stop import run as run_drive
from robot.JuliaFiles.ZombieSweep import run as run_sweep
from robot.robot import Robot

""" 
./ros2_ws/docker/restart.sh --> Restarts the Docker container
./ros2_ws/docker/enter_ros2.sh --> Enters the ROS2 environment

tail -f ros2_ws/runtime_output/robot_node.log --> Watches the robot node without starting it again

ros2 launch robot full_mission.launch.py --> launches all needed nodes for both scripts

"""

def run(robot: Robot) -> None:
    run_sweep(robot)
