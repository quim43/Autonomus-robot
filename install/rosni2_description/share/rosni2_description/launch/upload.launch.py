import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def declare_launch_arguments(launch_description: LaunchDescription):
    # Use sim time
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Defines if the models are loaded in simulation or in ",
    )
    launch_description.add_action(use_sim_time_arg)

    # Robot description URDF
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("rosni2_description"), "robot/rosni.urdf.xacro"]
    )
    urdf_xacro_path_arg = DeclareLaunchArgument(
        name="urdf_xacro_path",
        default_value=urdf_path,
        description="Defines if the models are loaded in simulation or in ",
    )

    launch_description.add_action(urdf_xacro_path_arg)


def declare_actions(launch_description: LaunchDescription):
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", LaunchConfiguration("urdf_xacro_path")]
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    launch_description.add_action(robot_state_publisher_node)


def generate_launch_description():
    ld = LaunchDescription()

    declare_launch_arguments(ld)
    declare_actions(ld)

    return ld
