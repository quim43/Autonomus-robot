from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

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
        self.look_ahead = 20

        # Initialize the TF2 buffer and listener to get transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define initial position and goal
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        self.matrix = None

        # Map Subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid, 
            "/map", 
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
        free = self.map_data == 0
        ocupied = self.map_data == 100
        
        ###########
        ### TODO:   Apply all needed Map transformatios to later plan safely the path
        ###########

        self.map_data = self.map_data.astype(np.uint8)

        self.map_data[unknowns] = 255
        self.map_data[free] = 255
        self.map_data[ocupied] = 0

        print("Map size: ", self.map_data.shape)

        # Convert from numpy array to python array
        self.matrix = self.map_data.tolist()

    def control_cb(self):
        """
        Control loop, explora el mapa i genera el path utilitzant A*, publica el punt
        més proper del path com a goal per al local planner.
        """
        if self.matrix == None:
            return
        # Update the grid graph
        
        # secure_map = self.mark_insecure_areas()

        # print(f"secure_map snippet: {secure_map[:10, :10]}")

        # print(f"secure_map: {secure_map}")
        # self.grid = Grid(matrix=secure_map.tolist())
        # print(f"self.grid: {self.grid}")

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

        # print("Grid Height", self.grid.height)
        # print("Grid Width", self.grid.width)

        # Verificar si los puntos inicial y final están en zonas seguras
        # if secure_map[p_start[1], p_start[0]] == 100 or secure_map[p_end[1], p_end[0]] == 100:
        #     self.get_logger().warn("Punto inicial o final en zona insegura. Recalculando...")
        #     return

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
        
        # Create the Solver object with A*
        path, runs = self.finder.find_path(start, end, self.grid)
        print(f"path: {path}")
        # print("Goal Pixel: ", self.map_data)
        # print("Robot Pixel: ", self.matrix)

        if len(path) == 0:
            self.get_logger().error("Path NOT found.")
            return

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

    def goal_cb(self, msg: Pose):
        self.get_logger().info("New goal recieved!")
        self.goal = msg
    
    def mark_insecure_areas(self):
        """
        Modifica el mapa para marcar como ocupadas las celdas dentro de una distancia segura
        que incluye el tamaño del robot (10 píxeles).
        """
        robot_size = 10  # Tamaño del robot en píxeles
        safe_distance_pixels = robot_size  # Usaremos directamente el tamaño del robot como distancia segura

        updated_map = self.map_data.copy()  # Copia del mapa para evitar modificar el original

        for y in range(self.height):
            for x in range(self.width):
                if self.map_data[y, x] == 0:  # Celda ocupada
                    # Marcar celdas alrededor del obstáculo como ocupadas
                    for dx in range(-safe_distance_pixels, safe_distance_pixels + 1):
                        for dy in range(-safe_distance_pixels, safe_distance_pixels + 1):
                            nx = x + dx
                            ny = y + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height:
                                updated_map[ny, nx] = 0  # Marcar como ocupado
        return updated_map



def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()