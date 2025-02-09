import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
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
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )
    launch_description.add_action(robot_state_publisher_launch)

    # Joint State Publisher gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    launch_description.add_action(joint_state_publisher_gui_node)

    # Rviz
    rviz_config_path = os.path.join(
        get_package_share_directory("rosni2_description"),
        "config/rviz2/default_view.rviz",
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
