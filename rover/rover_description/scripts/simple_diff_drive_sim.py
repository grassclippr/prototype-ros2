#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time

class SimpleDiffDriveSim(Node):
    def __init__(self):
        super().__init__('simple_diff_drive_sim')

        # Parameters
        self.declare_parameter('wheel_radius', 0.127)
        self.declare_parameter('wheel_separation', 0.66)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.w = 0.0

        self.left_pos = 0.0
        self.right_pos = 0.0

        # Publishers and TF
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)

        self.odom_pub = self.create_publisher(Odometry, 'diff_drive_controller/odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timer loop
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(0.02, self.update)  # 50 Hz

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Integrate pose
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # Compute wheel rotation
        v_left = self.v - self.w * self.wheel_separation / 2.0
        v_right = self.v + self.w * self.wheel_separation / 2.0
        w_left = v_left / self.wheel_radius
        w_right = v_right / self.wheel_radius

        self.left_pos += w_left * dt
        self.right_pos += w_right * dt

        # Publish joint_states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_wheel_motor_joint', 'right_wheel_motor_joint']
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [w_left, w_right]
        js.effort = [0.0, 0.0]
        self.joint_pub.publish(js)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = js.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = js.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDiffDriveSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
