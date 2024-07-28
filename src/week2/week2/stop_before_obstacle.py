import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class StopBeforeObstacle(Node):
    def __init__(self):
        super().__init__("stop_before_obstacle")
        self.velocity_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.minimum_obstacle_distance = 1.0
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.5
        self.get_logger().info("Stop Before Obstacle Node started.")

    def scan_callback(self, msg):
        nearest_obstacle_distance = np.round(np.min(np.array(msg.ranges)), decimals=1)
        if nearest_obstacle_distance <= self.minimum_obstacle_distance:
            self.vel_msg.linear.x = 0.0
            self.velocity_publisher_.publish(self.vel_msg)
            self.get_logger().info("Obstacle Detected Stopping.")
            rclpy.shutdown()
        self.velocity_publisher_.publish(self.vel_msg)


def main(args=None):
    rclpy.init(args=args)
    sto = StopBeforeObstacle()
    rclpy.spin(sto)
    sto.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
