#!/usr/bin/env python3

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class SendNavGoal(Node):
    def __init__(self) -> None:
        super().__init__("send_nav_goal")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a local NavigateToPose goal.")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw", type=float, nargs="?", default=0.0)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = SendNavGoal()
    if not node.client.wait_for_server(timeout_sec=args.timeout):
        print("FAIL: navigate_to_pose action server is not available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    pose = PoseStamped()
    pose.header.frame_id = args.frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    qx, qy, qz, qw = yaw_to_quaternion(args.yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    goal = NavigateToPose.Goal()
    goal.pose = pose
    send_future = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print("FAIL: goal was rejected")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"OK: goal accepted x={args.x:.2f} y={args.y:.2f} yaw={args.yaw:.2f} frame={args.frame}")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    print(f"result status={result.status}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.status == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
