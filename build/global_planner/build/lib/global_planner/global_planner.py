from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path

import numpy as np
import math
from geometry_msgs.msg import Pose, PoseStamped


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    roll is rotation around the x-axis
    pitch is rotation around the y-axis
    yaw is rotation around the z-axis
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner_node")

        # Definim parametres
        self.goal = Pose()
        self.current_pos = Pose()
        self.look_ahead = 5

        # Initialize the TF2 buffer and listener to get transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define initial position and goal
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        self.matrix = None

        # Map Subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid, 
            "/global_costmap/costmap", 
            self.map_cb, 
            10
        )

        # Subscriber al Goal
        self.goal_sub = self.create_subscription(
            Pose, 
            "/global_planner/goal", # procedent de explorer_node
            self.goal_cb, 
            10
        )

        self.path_pub = self.create_publisher(Path, "/global_path", 10)

        # Velocity publisher
        self.local_goal_pub = self.create_publisher(
            Pose, 
            "/local_planner/goal", 
            10
        )

        # Definim el control loop
        self.dt = 1.0

        # timer
        self.create_timer(
            self.dt, 
            self.control_cb
        )
        self.get_logger().info("Global Planner Node started.")
    '''
    def inflate_obstacles(self, map_data, inflation_radius, resolution):
        """
        Inflates the obstacles in the map using numpy operations.
        
        Parameters:
        - map_data: numpy array of the map (2D array).
        - inflation_radius: radius in meters to inflate the obstacles.
        - resolution: resolution of the map in meters per pixel.

        Returns:
        - Inflated map as a numpy array.
        """
        radius_in_pixels = int(inflation_radius / resolution)
        inflated_map = map_data.copy()

        # Get the indices of the obstacles
        obstacle_indices = np.argwhere(map_data == 0)

        for x, y in obstacle_indices:
            # Compute the range of indices to modify
            x_min = max(0, x - radius_in_pixels)
            x_max = min(map_data.shape[0], x + radius_in_pixels + 1)
            y_min = max(0, y - radius_in_pixels)
            y_max = min(map_data.shape[1], y + radius_in_pixels + 1)

            # Inflate the obstacles in this region
            inflated_map[x_min:x_max, y_min:y_max] = 0

        return inflated_map
    '''
    def map_cb(self, msg: OccupancyGrid):
        # Extract map data and dimensions
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_data = np.array(msg.data).reshape((self.height, self.width))

        # for row in range(self.height):
        #     for column in range(self.width):
        #         if self.map_data[row, column] == -1:
        #             self.map_data[row, column] = 0
        unknowns = self.map_data == -1
        free = self.map_data <= 80
        ocupied = self.map_data > 80
        
        ###########
        ### TODO:   Apply all needed Map transformatios to later plan safely the path
        ###########

        self.map_data = self.map_data.astype(np.uint8)

        self.map_data[unknowns] = 255
        self.map_data[free] = 255
        self.map_data[ocupied] = 0

        # Inflate obstacles
        # inflation_radius = 0.2
        # self.map_dataio = self.inflate_obstacles(self.map_data, inflation_radius, self.resolution)

        # Convert from numpy array to python array
        self.matrix = self.map_data.tolist()
        # print("matrix ", self.matrix)

    def control_cb(self):
        """
        Control loop, explora el mapa i genera el path utilitzant A*, publica el punt
        més proper del path com a goal per al local planner.
        """
        if self.matrix == None:
            return

        # Update the grid graph
        self.grid = Grid(matrix=self.matrix)

        # Get Robot current Pose
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            robot_x, robot_y, robot_th = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                quaternion_to_euler(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )[2],
            )
            # Convert the robot's position to map grid indices
            robot_grid_x = int((robot_x - self.origin[0]) / self.resolution - 0.5)
            robot_grid_y = int((robot_y - self.origin[1]) / self.resolution - 0.5)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().info(
                "Transform from base_footprint to laser_frame not found"
            )
            return

        p_start = (robot_grid_x, robot_grid_y)

        print(f"P_start: {p_start}")


        # Validar punto inicial
        # try:
        #     p_start = self.find_nearest_free_cell(p_start)
        #     print(f"P_start_val: {p_start}")
        # except ValueError as e:
        #     self.get_logger().error(f"No se pudo ajustar el punto inicial: {e}")
        #     return

        ###########
        ### TODO:  Compute Goal's coordinates
        ###########

        # goal position in meters
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y

        # goal position in pixels
        self.goal_grid_x = int((goal_x - self.origin[0]) / self.resolution - 0.5)
        self.goal_grid_y = int((goal_y - self.origin[1]) / self.resolution - 0.5)
        p_end = (self.goal_grid_x, self.goal_grid_y)
        
        print(f"P_end: {p_end}")

        # Validar punto final
        # try:
        #     p_end = self.find_nearest_free_cell(p_end, exclude_point=p_start)
        #     print(f"P_end_val: {p_end}")
        # except ValueError as e:
        #     self.get_logger().error(f"No se pudo ajustar el punto final: {e}")
        #     return

        # print("Grid Height", self.grid.height)
        # print("Grid Width", self.grid.width)

        # Define the start and ending nodes
        try:
            
            start = self.grid.node(p_start[0], p_start[1])
            end = self.grid.node(p_end[0], p_end[1])

            '''
            start = self.grid.node(50, 20)
            end = self.grid.node(70, 20)
            '''

            print(f"start: {start} , end: {end}")
        except:
            self.get_logger().error("Set node out of the map range")
            return
        # self.visualize_map(p_start, p_end)
        # Create the Solver object with A*
        path, runs = self.finder.find_path(start, end, self.grid)
        print(f"path: {path}")
        # print("Goal Pixel: ", self.map_data)
        # print("Robot Pixel: ", self.matrix)

        if len(path) == 0:
            self.get_logger().error("Path NOT found.")
            return

            '''
            try:
                max_attempts = 10  # Número máximo de intentos
                attempt = 0
                path_found = False

                while attempt < max_attempts and not path_found:
                    # Buscar un nuevo punto final cercano
                    p_end = self.find_nearest_free_cell(p_end, exclude_point=p_start)
                    self.get_logger().info(f"Attempt {attempt + 1}: Adjusted goal point to: {p_end}")

                    # Recalcular el camino con el nuevo punto final
                    end = self.grid.node(p_end[0], p_end[1])
                    path, runs = self.finder.find_path(start, end, self.grid)

                    if len(path) > 0:
                        path_found = True
                        self.get_logger().info("Path found after adjustment.")
                    else:
                        self.get_logger().warn(f"Attempt {attempt + 1}: Path not found. Trying again...")
                        
                        # Movimiento temporal del robot
                        try:
                            direction_x = p_end[0] - p_start[0]
                            direction_y = p_end[1] - p_start[1]

                            # Normalizar la dirección para moverse un paso
                            norm = max(abs(direction_x), abs(direction_y))  # Evitar dividir por cero
                            step_x = int(direction_x / norm) if norm != 0 else 0
                            step_y = int(direction_y / norm) if norm != 0 else 0

                            # Generar nueva posición temporal
                            temp_position = (p_start[0] + step_x, p_start[1] + step_y)

                            # Verificar que la nueva posición sea válida
                            if self.map_dataio[temp_position[1], temp_position[0]] == 255:
                                self.get_logger().info(f"Temporary position set to: {temp_position}")

                                # Mover el robot hacia la posición temporal
                                local_goal = Pose()
                                local_goal.position.x = self.origin[0] + (temp_position[0] + 0.5) * self.resolution
                                local_goal.position.y = self.origin[1] + (temp_position[1] + 0.5) * self.resolution
                                self.local_goal_pub.publish(local_goal)

                                # Actualizar posición inicial para el próximo intento
                                p_start = temp_position
                            else:
                                self.get_logger().warn("Temporary position is blocked. Skipping movement.")
                        except Exception as e:
                            self.get_logger().error(f"Error during temporary movement: {e}")

                    attempt += 1

                if not path_found:
                    raise ValueError("Path still not found after maximum adjustments.")

            except ValueError as e:
                self.get_logger().error(f"Final path not found: {e}")
                # self.visualize_map(p_start, p_end)
                return
        '''

        # Convert Path to ROS PTAH Message
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for p in path:
        
            ###########
            ### TODO:   Create a ROS PATH from the computed path with A*
            ###########
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            pose_stamped.pose.position.x = self.origin[0] + (p.x + 0.5) * self.resolution
            pose_stamped.pose.position.y = self.origin[1] + (p.y + 0.5) * self.resolution

            ros_path.poses.append(pose_stamped)

        # Publish path
        self.path_pub.publish(ros_path)
        self.get_logger().info(f"Path published with {len(path)} poses.")
        
        # Publish next goal to local planner
        local_goal = Pose()
        if len(path) <= self.look_ahead:
            path_point = path[-1]
        else:
            path_point = path[self.look_ahead]

        local_goal.position.x = self.origin[0] + (path_point.x + 0.5) * self.resolution
        local_goal.position.y = self.origin[1] + (path_point.y + 0.5) * self.resolution

        self.local_goal_pub.publish(local_goal)

    def visualize_map(self, start, goal):
        # Voltear el mapa tanto horizontal como verticalmente
        flipped_map = np.flip(self.map_data, axis=(0, 1))  # Flip vertical y horizontal

        # Ajustar las posiciones de los puntos al mapa volteado
        flipped_start = (flipped_map.shape[1] - start[0] - 1, flipped_map.shape[0] - start[1] - 1)
        flipped_goal = (flipped_map.shape[1] - goal[0] - 1, flipped_map.shape[0] - goal[1] - 1)

        # Visualizar el mapa y los puntos
        plt.imshow(flipped_map, cmap='gray')
        plt.scatter(flipped_start[0], flipped_start[1], c='blue', label='Start')  # Punto inicial
        plt.scatter(flipped_goal[0], flipped_goal[1], c='red', label='Goal')  # Punto final
        plt.legend()
        plt.title("Mapa Inflado con Inicio y Objetivo (Flip H/V)")
        plt.show()


    def goal_cb(self, msg: Pose):
        self.get_logger().info("New goal recieved!")
        self.goal = msg
        
    '''
    def find_nearest_free_cell(self, point, max_search_radius=50,exclude_point=None):
        print(f"point: {point}")
        """
        Encuentra la celda libre más cercana a un punto dado.
        
        Parameters:
        - map_data: numpy array del mapa inflado.
        - point: (x, y) coordenadas del punto a verificar.
        - max_search_radius: radio máximo de búsqueda en celdas.

        Returns:
        - (x, y) coordenadas de la celda libre más cercana.
        """
        from queue import Queue

        if not isinstance(point, (tuple, list)) or len(point) != 2:
            raise ValueError(f"El punto debe ser una tupla (x, y). {point}")

        x_start, y_start = point
        if self.map_data[y_start, x_start] == 255 and point != exclude_point:  # Libre
            return point

        # Cola para búsqueda BFS
        q = Queue()
        q.put((x_start, y_start))
        visited = set()
        visited.add((x_start, y_start))

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4 direcciones cardinales

        while not q.empty():
            x, y = q.get()

            # Iterar sobre celdas vecinas
            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                # Verificar que la celda esté dentro de los límites
                if 0 <= nx < self.map_data.shape[1] and 0 <= ny < self.map_data.shape[0]:
                    if (nx, ny) not in visited:
                        if self.map_data[ny, nx] == 255:  # Libre
                            point = (nx, ny)
                            print("point", point)
                            return  point # Celda libre encontrada
                        visited.add((nx, ny))
                        q.put((nx, ny))

            # Si supera el radio máximo, salimos
            if abs(x - x_start) > max_search_radius or abs(y - y_start) > max_search_radius:
                break

        raise ValueError("No se encontró una celda libre cercana.")
'''


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()