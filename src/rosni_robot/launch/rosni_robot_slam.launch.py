import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def declare_actions(launch_description: LaunchDescription):
    # Start nav2 bringup
    hw_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                )
            ]
        ),
    )
    launch_description.add_action(hw_interface)

    # Start the SLAM toolbox
    hw_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        ),
    )
    launch_description.add_action(hw_interface)

    # Rviz
    rviz_config_path = os.path.join(
        get_package_share_directory("rosni_robot"),
        "config/rviz2/navigation.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )
    launch_description.add_action(rviz_node)


def generate_launch_description():
    ld = LaunchDescription()

    declare_actions(ld)

    return ld
