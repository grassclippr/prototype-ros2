#!/usr/bin/env python3

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCalibrationCheck(Node):
    def __init__(self, topic: str, timeout_sec: float) -> None:
        super().__init__("odom_calibration_check")
        self.first = None
        self.last = None
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        self.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, msg: Odometry) -> None:
        if self.first is None:
            self.first = msg
        self.last = msg

    def done(self) -> bool:
        return self.get_clock().now().nanoseconds / 1e9 >= self.deadline


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize odom displacement over a short manual calibration run.")
    parser.add_argument("--topic", default="/diff_drive_controller/odom")
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = OdomCalibrationCheck(args.topic, args.duration)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    first = node.first
    last = node.last
    node.destroy_node()
    rclpy.shutdown()

    if first is None or last is None:
        print(f"FAIL: no odometry received on {args.topic}")
        return 1

    dx = last.pose.pose.position.x - first.pose.pose.position.x
    dy = last.pose.pose.position.y - first.pose.pose.position.y
    distance = math.hypot(dx, dy)
    dyaw = yaw_from_quaternion(last.pose.pose.orientation) - yaw_from_quaternion(first.pose.pose.orientation)
    dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
    twist: Twist = last.twist.twist
    print(f"OK: observed {distance:.3f} m displacement and {dyaw:.3f} rad yaw change over {args.duration:.1f}s")
    print(f"last twist vx={twist.linear.x:+.3f} m/s wz={twist.angular.z:+.3f} rad/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
