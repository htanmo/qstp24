import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random


class GenerateIntegerPublisher(Node):
    def __init__(self):
        super().__init__("integer_generator")
        self.publisher_ = self.create_publisher(Int32, "integers", 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.generate_random_number)
        self.get_logger().info("Generate Integer Publisher started.")

    def generate_random_number(self):
        msg = Int32()
        msg.data = random.randint(0, 100)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = GenerateIntegerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
