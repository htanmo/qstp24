import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from path_planning import PathPlanning
from pid_controller import PIDController


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        self.subscription_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.publisher_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.path_planner = PathPlanning(
            grid=[
                [0, 0, 0, 0, 0],
                [1, 1, 1, 0, 1],
                [0, 0, 0, 0, 0],
                [0, 1, 0, 1, 0],
                [0, 0, 0, 0, 0],
            ],
            start=(0, 0),
            goal=(4, 4),
        )
        self.pid_controller = PIDController(kp=1.0, ki=0.0, kd=0.1)

        self.path = self.path_planner.a_star()

        if self.path:
            self.current_waypoint_index = 0
            self.update_goal()

    def update_goal(self):
        if self.current_waypoint_index < len(self.path):
            goal = self.path[self.current_waypoint_index]
            self.pid_controller.update_goal(goal[0], goal[1])

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.pid_controller.update_position(pose)

        distance, control_signal = self.pid_controller.control()

        if distance < 0.1:  # Close enough to the current waypoint
            self.current_waypoint_index += 1
            self.update_goal()

        cmd_msg = Twist()
        cmd_msg.linear.x = min(0.2 * distance, 0.5)  # Limit the speed
        cmd_msg.angular.z = control_signal
        self.publisher_cmd_vel.publish(cmd_msg)

        if self.current_waypoint_index >= len(self.path):
            self.get_logger().info("Goal reached!")
            self.publisher_cmd_vel.publish(Twist())  # Stop the robot


def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
