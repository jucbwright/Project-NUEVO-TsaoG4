from __future__ import annotations

from robot.maindrive_start_stop import run as run_drive
from robot.JuliaFiles.ZombieSweep import run as run_sweep
from robot.robot import Robot

"""
HOW TO RUN main.py (do this every time, in this order)
=======================================================
1. ./ros2_ws/docker/restart.sh   --> Restart the Docker container after editing source
2. ./ros2_ws/docker/enter_ros2.sh --> Enter the ROS2 environment (shell inside container)
3. ros2 launch robot full_mission.launch.py
       --> Launches bridge, RPLidar, GPS/ArUco sensors, vision, AND this robot
           mission node (main.py) together. Use this instead of running
           `ros2 run robot robot` on its own, otherwise the vision node
           (and therefore /vision/detections + latest.jpg) will not be running.

Other useful commands:
tail -f ros2_ws/runtime_output/robot_node.log --> Watch the robot node log without restarting it
"""

def run(robot: Robot) -> None:
    run_sweep(robot)
