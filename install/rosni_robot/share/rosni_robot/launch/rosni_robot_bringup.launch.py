import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def declare_actions(launch_description: LaunchDescription):
    # Robot state publisher (URDF model and frames)
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("rosni2_description"),
                    "launch",
                    "upload.launch.py",
                )
            ]
        ),
    )
    launch_description.add_action(robot_state_publisher_launch)

    # Start the ROS2 controllers
    hw_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("garrinator_hw_interface"),
                    "launch",
                    "hw_interface.launch.py",
                )
            ]
        ),
    )
    launch_description.add_action(hw_interface)

    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("garrinator_diff_drive_controller"),
                    "launch",
                    "diff_drive_controller.launch.py",
                )
            ]
        ),
    )
    launch_description.add_action(diff_drive_controller)

    # Start the YDLidar node
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ydlidar_ros2_driver"),
                    "launch",
                    "ydlidar_launch.py",
                )
            ]
        ),
    )
    launch_description.add_action(ydlidar)


def generate_launch_description():
    ld = LaunchDescription()

    declare_actions(ld)

    return ld
