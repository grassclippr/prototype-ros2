#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class WheelVelocityOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("wheel_velocity_odom_bridge")
        self.declare_parameter("input_topic", "/wheel_velocities")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("linear_variance", 0.05)
        self.declare_parameter("angular_variance", 0.08)

        self._input_topic = self.get_parameter("input_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._linear_variance = float(self.get_parameter("linear_variance").value)
        self._angular_variance = float(self.get_parameter("angular_variance").value)

        odom_topic = self.get_parameter("odom_topic").value
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(Twist, self._input_topic, self._twist_cb, 20)

        self._last_time = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    def _twist_cb(self, msg: Twist) -> None:
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self._yaw = math.atan2(
            math.sin(self._yaw + angular_z * dt),
            math.cos(self._yaw + angular_z * dt),
        )
        self._x += linear_x * math.cos(self._yaw) * dt
        self._y += linear_x * math.sin(self._yaw) * dt

        qx, qy, qz, qw = yaw_to_quaternion(self._yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist = msg
        odom.pose.covariance[0] = self._linear_variance
        odom.pose.covariance[7] = self._linear_variance
        odom.pose.covariance[35] = self._angular_variance
        odom.twist.covariance[0] = self._linear_variance
        odom.twist.covariance[7] = self._linear_variance
        odom.twist.covariance[35] = self._angular_variance
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelVelocityOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
