import math
import time
import buildhat
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('buildhat_differential_drive')
        self.declare_parameter('wheel_radius', .05)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.declare_parameter('wheel_separation', .10)
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
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
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 1)
        self.base_link_odom_broadcaster = TransformBroadcaster(self)
        polling_frequency = 0.1
        self.timer = self.create_timer(polling_frequency, self.timer_callback)
        self.motors = buildhat.MotorPair(self.left_wheel_name, self.right_wheel_name)
        self.motors.set_speed_unit_rpm(True)
        self.pose = [0.0, 0.0, 0.0]
        self.last_left_motor_position = self.motors._leftmotor.get_position()
        self.last_right_motor_position = self.motors._rightmotor.get_position()
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.last_msg_timestamp = time.time()

    def cmd_callback(self, msg):
        self.last_msg_timestamp = time.time()
        linear_x = msg.twist.linear.x   # Forward/backward speed
        angular_z = msg.twist.angular.z # Rotation speed
        left_radps = linear_x/self.wheel_radius + angular_z * self.wheel_separation * .5 / self.wheel_radius
        right_radps = linear_x/self.wheel_radius - angular_z * self.wheel_separation * .5 / self.wheel_radius
        left_rpm = self.left_wheel_radius_multiplier * left_radps * 60 / (2*math.pi)
        right_rpm = self.right_wheel_radius_multiplier * right_radps * 60 / (2*math.pi)
        limit = 100  # Suspect bug in buildhat library (confuses limits between percentage and rpm modes)
        # So we limit rpm's to prevent buildhat from crashing.
        # We record speed so we can use later in Odometry message (we don't get from buildhat as it does
        # not seem reliable or well documented).
        self.left_rpm = min(max(left_rpm, -limit), limit)
        self.right_rpm = min(max(right_rpm, -limit), limit)
        self.motors.start(self.left_rpm, self.right_rpm)

    def timer_callback(self):
        elapsed = time.time() - self.last_msg_timestamp
        if elapsed > self.cmd_vel_timeout:
            self.motors.stop()
            self.left_rpm = 0.0
            self.right_rpm = 0.0
        self.update_internal_odometry()
        self.send_odom_transform()
        self.send_odom_message()

    def update_internal_odometry(self):
        left_motor_position = self.motors._leftmotor.get_position()
        right_motor_position = self.motors._rightmotor.get_position()
        delta_left_motor_position = left_motor_position - self.last_left_motor_position
        delta_right_motor_position = right_motor_position - self.last_right_motor_position
        left_pos_radians = delta_left_motor_position/self.left_wheel_radius_multiplier * 2 * math.pi / 360
        right_pos_radians = delta_right_motor_position/self.right_wheel_radius_multiplier * 2 * math.pi / 360
        linear_x = self.wheel_radius * (left_pos_radians + right_pos_radians)/2
        angular_z = left_pos_radians * self.wheel_radius / self.wheel_separation - \
                right_pos_radians * self.wheel_radius / self.wheel_separation
        self.pose[0] += linear_x * math.cos(self.pose[2])
        self.pose[1] += linear_x * math.sin(self.pose[2])
        self.pose[2] += angular_z
        self.last_left_motor_position = left_motor_position
        self.last_right_motor_position = right_motor_position

    def send_odom_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.pose[2])
        rot = t.transform.rotation
        rot.x, rot.y, rot.z, rot.w = q[0], q[1], q[2], q[3]
        self.base_link_odom_broadcaster.sendTransform(t)

    def send_odom_message(self):
        left_speed_radians = self.left_rpm/self.left_wheel_radius_multiplier * 2 * math.pi / 60
        right_speed_radians = self.right_rpm/self.right_wheel_radius_multiplier * 2 * math.pi / 60
        linear_x_speed = self.wheel_radius * (left_speed_radians + right_speed_radians)/2
        angular_z_speed = left_speed_radians * self.wheel_radius / self.wheel_separation - \
                right_speed_radians * self.wheel_radius / self.wheel_separation
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = self.pose[0]
        odom_msg.pose.pose.position.y = self.pose[1]
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.pose[2])
        rot = odom_msg.pose.pose.orientation
        rot.x, rot.y, rot.z, rot.w = q[0], q[1], q[2], q[3]
        odom_msg.twist.twist.linear.x = linear_x_speed
        odom_msg.twist.twist.angular.z = angular_z_speed
        self.odom_publisher.publish(odom_msg)


rclpy.init()
differential_drive_node = DifferentialDriveNode()
rclpy.spin(differential_drive_node)
differential_drive_node.destroy_node()
rclpy.shutdown()
