from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        # Arduino bridge + web UI
        Node(
            package="bridge",
            executable="bridge",
            name="bridge",
            output="screen",
        ),

        # RPLidar C1 — obstacle avoidance (maindrive_start_stop)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1.launch.py"]
                )
            ),
            launch_arguments={
                "serial_port":      "/dev/rplidar",
                "serial_baudrate":  "460800",
                "frame_id":         "laser",
                "angle_compensate": "true",
            }.items(),
        ),

        # GPS / ArUco tag sensor — position fusion (maindrive_start_stop)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution(
        #             [FindPackageShare("sensors"), "launch", "sensors.launch.py"]
        #         )
        #     ),
        # ),

        # Vision node (debug) — traffic light, stop sign, zombie detection
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution(
        #             [FindPackageShare("vision"), "launch", "vision_debug.launch.py"]
        #         )
        #     ),
        # ),

        # Robot mission node — runs main.py (drive phase → zombie sweep)
        # Node(
        #     package="robot",
        #     executable="robot",
        #     name="robot",
        #     output="screen",
        # ),
        

    ])
