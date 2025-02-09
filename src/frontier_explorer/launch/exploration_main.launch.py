from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
    ])
