#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import Float64

class SimpleDiffDriveHardwareSim(Node):
    def __init__(self):
        super().__init__('simple_diff_drive_hw_sim')

        # Parameters
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.joint_names = ['left_wheel_motor_joint', 'right_wheel_motor_joint']

        # Subscribe to wheel velocity commands from diff_drive_controller
        self.left_sub = self.create_subscription(
            Float64,
            '/left_wheel_motor_joint/velocity/command',
            self.left_wheel_cmd_callback,
            10)

        self.right_sub = self.create_subscription(
            Float64,
            '/right_wheel_motor_joint/velocity/command',
            self.right_wheel_cmd_callback,
            10)

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.left_pos = 0.0
        self.right_pos = 0.0

    def left_wheel_cmd_callback(self, msg):
        self.left_wheel_vel = msg.data

    def right_wheel_cmd_callback(self, msg):
        self.right_wheel_vel = msg.data

    def update(self):
        dt = 0.02
        self.left_pos += self.left_wheel_vel * dt
        self.right_pos += self.right_wheel_vel * dt

        # Publish joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [self.left_wheel_vel, self.right_wheel_vel]
        self.joint_state_pub.publish(js)

        # Publish TF from odom → base_link (basic example)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDiffDriveHardwareSim()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
