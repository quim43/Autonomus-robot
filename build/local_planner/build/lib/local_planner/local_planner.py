import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Pose, TwistStamped, Twist


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


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_planner_node")

        # Definim parametres
        self.goal = Pose()
        self.current_pos = Pose()
        self.current_vel = Twist()

        # Threshold per aturar el control
        self.d_threshold = 0.1

        # Error en orientacio
        self.e = 0

        # Derivada de l'error en orientacio
        self.de = 0

        # Constant proporcional i derivativa del controlador PD
        self.kp = 0.8
        self.kd = 0.1

        # Subscriber to Odom
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # Subscriber al Goal
        self.goal_sub = self.create_subscription(
            Pose, "/local_planner/goal", self.goal_cb, 10
        )

        # Velocity publisher
        self.twist_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )

        # Definim el control loop
        self.dt = 0.1
        self.create_timer(self.dt, self.control_cb)

    def control_cb(self):
        """
        Control loop, aplica el controlador PD per generar commandes de velocitat i moure
        el robot de la posició inicial a la posició Goal.
        """
        # Condició per aplicar el control:
        d = math.sqrt(
            (self.goal.position.x - self.current_pos.position.x) ** 2 +
            (self.goal.position.y - self.current_pos.position.y) ** 2
        )
        self.get_logger().info(f"Distance to goal: {d}")
        
        # Si encara estem massa lluny del goal
        if d > self.d_threshold:
            # Pas 1: Definir pendent de la linia recta que uneix els punts
            phi_m = math.atan2(
                self.goal.position.y - self.current_pos.position.y,
                self.goal.position.x - self.current_pos.position.x
            )

            # Pas 2: Calcul error entre orientació del Robot (theta) i l'angle anterior
            _, _, theta = quaternion_to_euler(
                self.current_pos.orientation.x,
                self.current_pos.orientation.y,
                self.current_pos.orientation.z,
                self.current_pos.orientation.w
            )
            # self.e = phi_m - theta
            self.e = (phi_m - theta + math.pi) % (2 * math.pi) - math.pi # normalitzo el error


            # Pas 3: Calculem la derivada de l'error
            x = self.current_pos.position.x
            y = self.current_pos.position.y
            xd = self.goal.position.x
            yd = self.goal.position.y
            vel_y = self.current_vel.linear.y
            vel_x = self.current_vel.linear.x
            dphi_m = ((x-xd)*vel_y-(y-yd)*vel_x)/((x-xd)**2+(y-yd)**2)
            self.de = dphi_m - self.current_vel.angular.z

            # Pas final: calculem i publiquem el twist
            w_robot = self.kp * self.e + self.kd * self.de
            v_robot = 0.1  # Velocitat lineal constant

            twist = TwistStamped()
            twist.twist.linear.x = v_robot
            twist.twist.angular.z = w_robot
            self.twist_pub.publish(twist)

    def odom_cb(self, msg: Odometry):
        self.current_pos = msg.pose.pose
        self.current_vel = msg.twist.twist

    def goal_cb(self, msg: Pose):
        self.get_logger().info("New goal recieved!")
        self.goal = msg


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
