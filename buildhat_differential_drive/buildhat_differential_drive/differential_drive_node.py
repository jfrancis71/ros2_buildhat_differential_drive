import math
import time
import buildhat
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('buildhat_differential_drive')
        self.declare_parameter('wheel_radius', .05)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.declare_parameter('wheel_seperation', .10)
        self.wheel_seperation = self.get_parameter('wheel_seperation').get_parameter_value().double_value
        # Note your cmd_vel_timeout will not be effective if less than polling_frequency
        self.declare_parameter('cmd_vel_timeout', .5)
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.declare_parameter('left_wheel_name', "A")
        self.left_wheel_name = self.get_parameter('left_wheel_name').get_parameter_value().string_value
        self.declare_parameter('right_wheel_name', "D")
        self.right_wheel_name = self.get_parameter('right_wheel_name').get_parameter_value().string_value
        self.declare_parameter('left_wheel_radius_multiplier', 1.0)
        self.left_wheel_radius_multiplier = self.get_parameter('left_wheel_radius_multiplier').get_parameter_value().double_value
        self.declare_parameter('right_wheel_radius_multiplier', 1.0)
        self.right_wheel_radius_multiplier = self.get_parameter('right_wheel_radius_multiplier').get_parameter_value().double_value
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_callback,
            1
        )
        polling_frequency = 0.1
        self.timer = self.create_timer(polling_frequency, self.timer_callback)
        self.motors = buildhat.MotorPair(self.left_wheel_name, self.right_wheel_name)
        self.last_msg_timestamp = time.time()
        self.motors.set_speed_unit_rpm(True)

    def cmd_callback(self, msg):
        self.last_msg_timestamp = time.time()
        linear_x = msg.twist.linear.x   # Forward/backward speed
        angular_z = msg.twist.angular.z # Rotation speed
        left_radps = linear_x/self.wheel_radius + angular_z * self.wheel_seperation * .5 / self.wheel_radius
        right_radps = linear_x/self.wheel_radius - angular_z * self.wheel_seperation * .5 / self.wheel_radius
        left_rpm = self.left_wheel_radius_multiplier * left_radps * 60 / (2*math.pi)
        right_rpm = self.right_wheel_radius_multiplier * right_radps * 60 / (2*math.pi)
        limit = 100  # Suspect bug in buildhat library (confuses limits between percentage and rpm modes)
        # So we limit rpm's to prevent buildhat from crashing.
        left_rpm = min(max(left_rpm, -limit), limit)
        right_rpm = min(max(right_rpm, -limit), limit)

        self.motors.start(left_rpm, right_rpm)

    def timer_callback(self):
        elapsed = time.time() - self.last_msg_timestamp
        if elapsed > self.cmd_vel_timeout:
            self.motors.stop()


rclpy.init()
differential_drive_node = DifferentialDriveNode()
rclpy.spin(differential_drive_node)
differential_drive_node.destroy_node()
rclpy.shutdown()
