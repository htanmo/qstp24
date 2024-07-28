import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sqrt

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.5
        self.goal_distance = 5.0 # Metres
        self.get_logger().info("Move To Goal Node started.")
    
    def odom_callback(self, odom_data):
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        current_distance = sqrt(x**2 + y**2)
        if current_distance >= self.goal_distance:
            self.vel_msg.linear.x = 0.0
            self.publisher_.publish(self.vel_msg)
            self.get_logger().info("Goal Reached.")
            rclpy.shutdown()
        else:
            self.vel_msg.linear.x = 5.0
            self.publisher_.publish(self.vel_msg)


def main(args=None):
    rclpy.init(args=args)
    mtg = MoveToGoal()
    rclpy.spin(mtg)
    mtg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()