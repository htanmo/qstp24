import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class PIDController(Node):
    def __init__(self, kp, ki, kd):
        super().__init__("pid_controller")
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.goal_position = None
        self.previous_error = 0.0
        self.integral = 0.0

    def update_goal(self, goal_x, goal_y):
        self.goal_position = {"x": goal_x, "y": goal_y}

    def control(self):
        error_x = self.goal_position["x"] - self.current_position["x"]
        error_y = self.goal_position["y"] - self.current_position["y"]

        distance = math.sqrt(error_x**2 + error_y**2)
        angle_to_goal = math.atan2(error_y, error_x)

        error_theta = angle_to_goal - self.current_position["theta"]
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

        P = self.Kp * error_theta
        self.integral += error_theta
        I = self.Ki * self.integral
        derivative = error_theta - self.previous_error
        D = self.Kd * derivative

        self.previous_error = error_theta

        control_signal = P + I + D
        return distance, control_signal

    def update_position(self, pose: Pose):
        position = pose.position
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self.current_position["x"] = position.x
        self.current_position["y"] = position.y
        self.current_position["theta"] = theta
