import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 

class  MoveTurtleBot(Node):
    def __init__(self):
        super().__init__('move_turtlebot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.5
        self.get_logger().info("Move TurtleBot Publisher started.")
    
    def timer_callback(self):
        self.publisher_.publish(self.vel_msg)


def main(args=None):
    rclpy.init(args=args)
    move_turtlebot = MoveTurtleBot()
    rclpy.spin(move_turtlebot)
    move_turtlebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()