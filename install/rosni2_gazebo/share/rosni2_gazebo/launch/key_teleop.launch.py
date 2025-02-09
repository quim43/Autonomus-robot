from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def declare_actions(launch_description: LaunchDescription):
    key_teleop_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "ros2",
            "run",
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
            "--ros-args",
            "-p",
            "stamped:=true",
            "--remap",
            "/cmd_vel:=/diff_drive_controller/cmd_vel",
        ],
        output="screen",
    )
    launch_description.add_action(key_teleop_node)


def generate_launch_description():
    ld = LaunchDescription()

    declare_actions(ld)

    return ld
