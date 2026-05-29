"""
Launch file for the Global GPS stack on the Jetson Nano.

Starts:
  1. realsense2_camera  — RealSense D4xx driver (colour + aligned depth)
  2. ground_localizer   — ArUco field localizer, publishes /global_gps/tag_detections

Usage (inside the Jetson Docker container):
    ros2 launch global_gps global_gps.launch.py
    ros2 launch global_gps global_gps.launch.py marker_size:=0.15 corner_ids:=[0,1,2,3]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ── Launch arguments ──────────────────────────────────────────────────
    marker_size_arg = DeclareLaunchArgument(
        "marker_size",
        default_value="0.10",
        description="Physical side length of ArUco markers in metres.",
    )
    corner_ids_arg = DeclareLaunchArgument(
        "corner_ids",
        default_value="[0, 1, 2, 3]",
        description="Marker IDs used as fixed field-corner anchors.",
    )
    rover_ids_arg = DeclareLaunchArgument(
        "rover_ids",
        default_value="[11, 12, 13, 14, 15, 16, 17, 18]",
        description="Marker IDs that appear on rovers.",
    )
    tcp_port_arg = DeclareLaunchArgument(
        "tcp_port",
        default_value="7777",
        description="TCP port for the robot push server (NAT-friendly delivery).",
    )

    # ── Ground localizer node ─────────────────────────────────────────────
    # NOTE: realsense2_camera must be started separately before launching this.
    # Run: ros2 launch realsense2_camera rs_launch.py
    localizer_node = Node(
        package="global_gps",
        executable="ground_localizer",
        name="ground_localizer",
        output="screen",
        parameters=[{
            "marker_size": LaunchConfiguration("marker_size"),
            "corner_ids": LaunchConfiguration("corner_ids"),
            "rover_ids": LaunchConfiguration("rover_ids"),
            "tcp_port": LaunchConfiguration("tcp_port"),
        }],
    )

    return LaunchDescription([
        marker_size_arg,
        corner_ids_arg,
        rover_ids_arg,
        tcp_port_arg,
        localizer_node,
    ])
