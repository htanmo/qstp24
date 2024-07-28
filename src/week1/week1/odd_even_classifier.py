import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class OddEvenClassifier(Node):
    def __init__(self):
        super().__init__("odd_even_classifier")
        self.subscription = self.create_subscription(
            Int32, "integers", self.listener_callback, 10
        )
        self.publisher_ = self.create_publisher(String, "oddeven", 10)
        self.get_logger().info("Even Odd Classifier Publisher started.")

    def listener_callback(self, msg):
        result = "odd" if msg.data & 1 else "even"
        new_msg = String()
        new_msg.data = f"{msg.data} is {result}"
        self.publisher_.publish(new_msg)
        self.get_logger().info(f"Publishing: {new_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = OddEvenClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
