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
        default_value="empty",
        description="Defines the world of gazebo to be loaded",
    )
    launch_description.add_action(world_arg)

    # Enable gui
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Start the GUI client",
    )
    launch_description.add_action(gui_arg)


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
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    launch_description.add_action(robot_state_publisher_launch)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    def generate_world_path(context):
        # Get the 'world' argument value from the context
        world_name = LaunchConfiguration("world").perform(context)
        # Append '.world' extension to the world name
        world_file = world_name + ".world"

        # Check Gui
        use_gui = LaunchConfiguration("gui").perform(context)

        # Construct the full path to the world file
        pkg_robot_description = get_package_share_directory("rosni2_gazebo")
        world_path = os.path.join(pkg_robot_description, "worlds", world_file)

        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    )
                ]
            ),
            launch_arguments={
                "world": world_path,
                "verbose": "true",
                "gui": use_gui,
            }.items(),
        )
        return [gazebo_launch]

    launch_description.add_action(OpaqueFunction(function=generate_world_path))

    # Spawn the Rosni robot in gazebo
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "rosni_robot",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )
    launch_description.add_action(spawn_entity_node)

    # Start the ROS2 controllers
    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    launch_description.add_action(joint_state_broadcaster)

    diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_controller",
        ],
        output="screen",
    )
    launch_description.add_action(diff_drive_controller)


def generate_launch_description():
    ld = LaunchDescription()

    declare_launch_arguments(ld)
    declare_actions(ld)

    return ld
