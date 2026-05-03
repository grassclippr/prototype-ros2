#!/usr/bin/env python3

import argparse
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationCheck(Node):
    def __init__(self, odom_topic: str, timeout_sec: float) -> None:
        super().__init__("localization_check")
        self.odom = None
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def timed_out(self) -> bool:
        return self.get_clock().now().nanoseconds / 1e9 >= self.deadline


def main() -> int:
    parser = argparse.ArgumentParser(description="Check filtered odometry and localization TF.")
    parser.add_argument("--odom-topic", default="/odometry/filtered")
    parser.add_argument("--parent-frame", default="odom")
    parser.add_argument("--child-frame", default="base_link")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationCheck(args.odom_topic, args.timeout)
    tf_ok = False
    while rclpy.ok() and not node.timed_out():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            node.tf_buffer.lookup_transform(args.parent_frame, args.child_frame, rclpy.time.Time())
            tf_ok = True
        except TransformException:
            pass
        if node.odom is not None and tf_ok:
            break

    odom = node.odom
    node.destroy_node()
    rclpy.shutdown()

    if odom is None:
        print(f"FAIL: no odometry received on {args.odom_topic}")
        return 1
    if not tf_ok:
        print(f"FAIL: missing TF {args.parent_frame} -> {args.child_frame}")
        return 1

    p = odom.pose.pose.position
    t = odom.twist.twist
    print(f"OK: {args.odom_topic} and TF {args.parent_frame}->{args.child_frame} are available")
    print(f"pose x={p.x:+.3f} y={p.y:+.3f}; twist vx={t.linear.x:+.3f} wz={t.angular.z:+.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
