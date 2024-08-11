import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt


class PIDController(Node):
    def __init__(self):
        super().__init__("pid_controller")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        # PID constants
        self.kp_linear = 0.5
        self.ki_linear = 0.5
        self.kd_linear = 0.5

        self.kp_angular = 0.5
        self.ki_angular = 0.5
        self.kd_angular = 0.5

        # Errors for PID
        self.previous_error_linear = 0.0
        self.previous_error_angular = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0

    def set_goal(self, x, y):
        self.goal_x = x
        self.goal_y = y

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        self.yaw = atan2(
            2.0
            * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
            1.0
            - 2.0
            * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z),
        )
        self.control_loop()

    def control_loop(self):
        # Calculate errors
        distance = sqrt(
            (self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2
        )
        angle_to_goal = atan2(
            self.goal_y - self.current_y, self.goal_x - self.current_x
        )
        angle_error = angle_to_goal - self.yaw

        # PID for linear velocity
        self.integral_linear += distance
        derivative_linear = distance - self.previous_error_linear
        linear_speed = (
            self.kp_linear * distance
            + self.ki_linear * self.integral_linear
            + self.kd_linear * derivative_linear
        )

        # PID for angular velocity
        self.integral_angular += angle_error
        derivative_angular = angle_error - self.previous_error_angular
        angular_speed = (
            self.kp_angular * angle_error
            + self.ki_angular * self.integral_angular
            + self.kd_angular * derivative_angular
        )

        self.previous_error_linear = distance
        self.previous_error_angular = angle_error

        # Publish the velocities
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()

    # Goal
    pid_controller.set_goal(1.0, 1.0)

    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
