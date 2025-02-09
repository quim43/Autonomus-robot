from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #for inicialize the second launch file
    simulation_launch_file = os.path.join(
        get_package_share_directory('rosni2_gazebo'),
        'launch',
        'rosni_slam.launch.py'
    )

    return LaunchDescription([
        # Nodo de Explorer
        Node(
            package='frontier_explorer',  # Reemplaza con el nombre del paquete donde está el nodo
            executable='explorer_node',  # Nombre del ejecutable del nodo
            name='explorer_node',
            output='screen'
        ),
        # Nodo de Global Planner
        Node(
            package='global_planner',  # Reemplaza con el nombre del paquete donde está el nodo
            executable='global_planner_node',  # Nombre del ejecutable del nodo
            name='global_planner_node',
            output='screen'
        ),
        # Nodo de Local Planner
        Node(
            package='local_planner',  # Reemplaza con el nombre del paquete donde está el nodo
            executable='local_planner_node',  # Nombre del ejecutable del nodo
            name='local_planner_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch_file)
        )
    ])