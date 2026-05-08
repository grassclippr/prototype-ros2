#!/usr/bin/env python3

import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish stamped cmd_vel commands")
    parser.add_argument("--topic", default="/diff_drive_controller/cmd_vel")
    parser.add_argument("--linear-x", type=float, default=0.2)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--frame-id", default="base_link")
    return parser.parse_args()


def publish_twist(
    node: Node,
    publisher,
    topic_frame_id: str,
    linear_x: float,
    angular_z: float,
) -> None:
    msg = TwistStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = topic_frame_id
    msg.twist.linear.x = linear_x
    msg.twist.angular.z = angular_z
    publisher.publish(msg)


def main() -> None:
    args = parse_args()
    period_sec = 1.0 / args.rate_hz

    rclpy.init()
    node = Node("wheel_cmd_vel_stamped")
    publisher = node.create_publisher(TwistStamped, args.topic, 10)

    try:
        end_time = time.monotonic() + args.duration_sec
        while time.monotonic() < end_time:
            publish_twist(node, publisher, args.frame_id, args.linear_x, args.angular_z)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_sec)

        publish_twist(node, publisher, args.frame_id, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
