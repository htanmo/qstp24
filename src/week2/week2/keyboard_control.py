import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Keyboard control node has been started.')
        self.running = True

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok() and self.running:
            key = self.get_key()
            if key == 'w':
                twist.linear.x = 0.5
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -0.5
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            elif key == 'q':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.running = False
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher_.publish(twist)
            self.get_logger().info(f'Publishing: linear={twist.linear.x}, angular={twist.angular.z}')

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def stop(self):
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    thread = threading.Thread(target=node.run)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        node.stop()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
