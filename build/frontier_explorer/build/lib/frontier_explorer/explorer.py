import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
import numpy as np
import math as m
from tf2_ros import TransformListener, Buffer

directions = [
    (-1,-1), (-1,0), (-1,1),
    (0,-1),          (0,1),
    (1,-1),  (1,0),  (1,1)
    ]

FREE = 0
UNDEFINED = -1

class Frontier:
    def __init__(self, frontier_id, initial_pixel):
        self.id = frontier_id  # Unique identifier for the frontier
        self.cells = [initial_pixel]  # List of (x, y) coordinates of frontier pixels
        self.num_cells = 1  # Number of cells in the frontier
        self.center_cell = initial_pixel  # Pick the first pixel as the center cell initially

    def add_cell(self, pixel):
        self.cells.append(pixel)
        self.num_cells += 1
        self.center_cell = self.compute_center_cell()

    def compute_center_cell(self):
        row_sum = 0
        column_sum = 0
        for pixel in self.cells:
            row_sum += pixel[0]
            column_sum += pixel[1]

        return  (
            int(row_sum/self.num_cells), 
            int(column_sum/self.num_cells)
        )

class FrontierDetectionNode(Node):

    '''
    
    Objectives of the Node:

    1. With the costmap, detect the frontiers and filter them
    2. Publish the validated frontiers 
    3. Select a goal in function of the implemented algorith
    4. Publish the goal for the global_planner
    
    '''

    def __init__(self):
        super().__init__("explorer_node")
        
        # for store and listen all the frames published
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscription to the costmap variation, this brings us a narrow map, strech streets
        self.map_subscription = self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.map_callback, 10)

        # once we have the frontiers, publish them for visualize
        self.frontier_publisher = self.create_publisher( MarkerArray, "/frontiers", 10)

        # this goal is for the global_planner
        self.goal_publisher = self.create_publisher(Pose, "/global_planner/goal",10)

        self.selection_mode = self.display_menu()
        self.get_logger().info(f"Selected mode: {self.selection_mode}")
        self.get_logger().info("Frontier Detection Node started.")

        # Store the initial robot position
        self.initial_position = None
        self.set_initial_position()
    
    def set_initial_position(self):
        while self.initial_position is None:
            robot_position = self.get_robot_position()
            if robot_position:
                self.initial_position = robot_position[:2]
                self.get_logger().info(f"Initial position set to: x={self.initial_position[0]}, y={self.initial_position[1]}")
            else:
                self.get_logger().warn("Waiting for initial position...")
                rclpy.spin_once(self, timeout_sec=1.0)

    def display_menu(self):
        print("\nSeleccione el modo de exploración:")
        print("1 - La frontera mes proxima")
        print("2 - La frontera més gran")
        print("3 - La millor frontera")
        while True:
            try:
                choice = int(input("Tria una opció, {1, 2 o 3}: "))
                if choice in [1, 2, 3]:
                    return choice
                else:
                    print("Escull una opció valida.")
            except ValueError:
                print("Entrada inválida.")

    def map_callback(self, msg: OccupancyGrid):

        '''

        1. Get the costmap metadata
        2. Call the find_frontier method for calculate the frontiers and store them in a list
        3. Publish this list for visualize the frontiers
        4. Select the next goal from the frontiers list
        
        '''

        # extract the metadata info from the costmap
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        # values goes from -1 to max 100.
        map_data = np.array(msg.data).reshape((height, width))

        frontier_map = np.zeros((height, width, 1), dtype=np.uint8) #matriu per identificar les fronteres totes juntes

        frontiers = self.find_frontiers(map_data, frontier_map, width, height)

        if not frontiers:
            self.end_frontiers()
        else:
            self.publish_frontiers(frontiers, msg.info)
            self.select_goal_frontier(frontiers, map_data, msg.info)


    def map_callback_map(self, msg: OccupancyGrid):
        """
        Callback for processing data from the /map topic.
        """
        # Procesar los datos del mapa según sea necesario
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

         # values goes from -1 to max 100.
        map_data = np.array(msg.data).reshape((height, width))

        frontier_map = np.zeros((height, width, 1), dtype=np.uint8) #matriu per identificar les fronteres totes juntes

        frontiers = self.find_frontiers(map_data, frontier_map, width, height)

        self.publish_frontiers(frontiers, msg.info)


    def find_frontiers( self, map_data: np.ndarray, frontier_map: np.ndarray, columns: int, rows: int) -> list:

        '''
        
        1. Search the pixels that delimitate a frontier.
        2. Connects these pixels that belong on the same frontier and creates de object
        3. Return a filtered frontier list
        
        '''

        # determina si un pixel forma part de una frontera
        def is_frontier(r, c):
            if map_data[r, c] == UNDEFINED:  # Píxel desconegut?
                for d in directions:
                    nr = r + d[0] #vei de fila
                    nc = c + d[1] #vei de columna
                    if 0 <= nr < rows and 0 <= nc < columns: # comprovació dels límits de la matriu
                        if 50 >= map_data[nr,nc] >= FREE: #el pixel veí és conegut?
                            return True
            return False

        #funció per detectar pixels en la mateixa frontera
        def connected_frontier(r: int, c: int, frontier: object):

            stored = [(r, c)] #starts with the input pixel
            while stored: #bucle fins que no quedin pixels en la llista stored
                cr, cc = stored.pop() #extracció d'un pixel
                if frontier_map[cr, cc] == 0:
                    frontier_map[cr, cc] = 255 #abans ja s'ha dit que es frontera, aqui es reflexa al mapa pertinent
                    frontier.add_cell((cc,cr))
                    for d in directions:
                        nr = cr + d[0]
                        nc = cc + d[1]
                        if (0 <= nr < rows and 0 <= nc < columns):
                            if is_frontier(nr, nc) and frontier_map[nr, nc] == 0:
                                stored.append((nr, nc))

        # List of Frontier objects (given class on top of the file)
        frontiers = []
        for row in range(rows):
            for column in range(columns):
                if(is_frontier(row, column)):
                    new_frontier = Frontier(frontier_id= len(frontiers)+1, initial_pixel = (column,row))
                    connected_frontier(row, column, new_frontier)
                    if new_frontier.num_cells > 3:
                        frontiers.append(new_frontier)
        return frontiers

    def get_robot_position(self):
        '''
        1. Perform a lookup_transform giving the two frames for reference
        2. With the returned data, get the position and rotation.
        '''
        
        try:
            # Verificar si la transformación está disponible
            if not self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0)):
                self.get_logger().warn("Transform 'map' -> 'base_footprint' not available yet. Waiting...")
                return None

            # Consultar la transformación
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                rclpy.time.Time(),
                timeout=rclpy.time.Duration(seconds=1.0)
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.rotation
            )
        except Exception as e:
            self.get_logger().error(f"Failed to get robot position: {e}")
            return None
    
    def nearest_frontier(self, frontiers: list, map_info: MapMetaData):

        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().warn("Robot position unavailable. Skipping nearest frontier selection.")
            return None
        
        x_robot, y_robot, _ = robot_position
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        min_distance = float('inf')
        selected_frontier = None

        for frontier in frontiers:
            center_x, center_y = frontier.center_cell
            world_x = origin_x + (center_x + 0.5) * resolution
            world_y = origin_y + (center_y + 0.5) * resolution
            distance = m.sqrt((x_robot - world_x) ** 2 + (y_robot - world_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                selected_frontier = frontier

        return selected_frontier

    def largest_frontier(self, frontiers: list):
        return max(frontiers, key=lambda f: f.num_cells * f.num_cells, default=None)
   
    def best_information_gain_frontier(self, frontiers: list, map_data: np.ndarray, map_info: MapMetaData):
        """
        Selects the frontier that maximizes information gain while minimizing movement cost.
        Combines:
        - Information gain (unknown cells around the frontier)
        - Movement cost (distance to the frontier)
        """
        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().warn("Robot position unavailable. Skipping information gain selection.")
            return None

        x_robot, y_robot, _ = robot_position
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        best_score = -float('inf')
        best_frontier = None

        for frontier in frontiers:
            center_x, center_y = frontier.center_cell
            world_x = origin_x + (center_x + 0.5) * resolution
            world_y = origin_y + (center_y + 0.5) * resolution

            # Calculate distance (movement cost)
            distance = m.sqrt((x_robot - world_x) ** 2 + (y_robot - world_y) ** 2)

            # Calculate information gain (unknown cells around the frontier)
            info_gain = sum(
                1 for cell in frontier.cells
                if self.is_unknown(map_data, cell[0], cell[1])
            )

            # Combine metrics: Higher info gain and lower distance are better
            score = info_gain - 0.5 * distance  # Adjust weight as needed

            if score > best_score:
                best_score = score
                best_frontier = frontier

        return best_frontier

    def is_unknown(self, map_data: np.ndarray, x: int, y: int):
        """
        Checks if a cell is unknown (-1) in the map.
        """
        # Asegúrate de que los índices están dentro de los límites del mapa
        if 0 <= y < map_data.shape[0] and 0 <= x < map_data.shape[1]:
            return map_data[y, x] == UNDEFINED
        return False


    
    def select_goal_frontier(self, frontiers: list, map_data: np.ndarray, map_info: MapMetaData):
        if self.selection_mode == 1:
            selected_frontier = self.nearest_frontier(frontiers, map_info)
        elif self.selection_mode == 2:
            selected_frontier = self.largest_frontier(frontiers)
        elif self.selection_mode == 3:  # Nuevo método
            selected_frontier = self.best_information_gain_frontier(frontiers, map_data, map_info)
        else:
            selected_frontier = None  # Placeholder for future mode

        if selected_frontier:
                
            next_goal=(
                selected_frontier.center_cell[0], 
                selected_frontier.center_cell[1]
            )

            global_goal = Pose()
            global_goal.position.x = self.origin[0] + (next_goal[0] + 0.5) * self.resolution
            global_goal.position.y = self.origin[1] + (next_goal[1] + 0.5) * self.resolution

            print("Next goal position:")
            print(f"X: {global_goal.position.x}   Y: {global_goal.position.y}")
            self.goal_publisher.publish(global_goal)

    def end_frontiers(self):
        if self.initial_position:
            global_goal = Pose()
            global_goal.position.x = self.initial_position[0]
            global_goal.position.y = self.initial_position[1]
            self.goal_publisher.publish(global_goal)
            self.get_logger().info(f"No more frontiers. Returning to initial position: x={self.initial_position[0]}, y={self.initial_position[1]}")

    def publish_frontiers(self, frontiers: list, map_info: MapMetaData):
        '''Publish the detected Frontiers to visualize it from RViz
        Arguments:
        - frontiers: List[Frontier] = a list of Frontier objects to be published
        - map_info
        '''
        marker_array = MarkerArray()
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        for frontier in frontiers:
            marker = Marker()
            marker.header.frame_id = "map"  # Use the map frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"
            marker.id = frontier.id  # Unique ID for each frontier
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Size of the points in meters
            marker.scale.y = 0.05
            marker.color.a = 1.0  # Alpha channel
            marker.color.r = np.random.random()
            marker.color.g = np.random.random()
            marker.color.b = np.random.random()

            # Add cells of the frontier to the marker
            for cell in frontier.cells:
                x = cell[0]
                y = cell[1]
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                #print(f"Cell x: {x}. Cell y: {y}")
                #print(f"World x: {world_x}. World y: {world_y}")
                point = Point()
                point.x = world_x
                point.y = world_y
                point.z = 0.0  # Frontiers are in 2D, so z=0
                marker.points.append(point)

            # Add this marker to the MarkerArray
            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.frontier_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()