import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def declare_launch_arguments(launch_description: LaunchDescription):
    # Use sim time
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Defines if the models are loaded in simulation or in ",
    )
    launch_description.add_action(use_sim_time_arg)

    # World
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value="maze",
        description="Defines the world of gazebo to be loaded",
    )
    launch_description.add_action(world_arg)

    # Enable gui
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false",
        description="Start the GUI client",
    )
    launch_description.add_action(gui_arg)


def declare_actions(launch_description: LaunchDescription):
    # Start nav2 bringup
    nav2 = IncludeLaunchDescription(
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
    launch_description.add_action(nav2)

    # Start the SLAM toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    launch_description.add_action(slam)

    # Rviz
    rviz_config_path = os.path.join(
        get_package_share_directory("rosni2_gazebo"),
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

    # Robot state publisher (URDF model and frames)
    rosni_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("rosni2_gazebo"),
                    "launch",
                    "rosni_gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )
    launch_description.add_action(rosni_gazebo)


def generate_launch_description():
    ld = LaunchDescription()

    declare_launch_arguments(ld)
    declare_actions(ld)

    return ld
