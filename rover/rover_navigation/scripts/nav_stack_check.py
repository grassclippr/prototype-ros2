#!/usr/bin/env python3

import argparse
import sys

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavStackCheck(Node):
    def __init__(self, timeout_sec: float) -> None:
        super().__init__("nav_stack_check")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timeout_sec = timeout_sec


def main() -> int:
    parser = argparse.ArgumentParser(description="Check whether the Nav2 NavigateToPose action is available.")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavStackCheck(args.timeout)
    ok = node.client.wait_for_server(timeout_sec=args.timeout)
    node.destroy_node()
    rclpy.shutdown()

    if not ok:
        print("FAIL: navigate_to_pose action server is not available")
        return 1
    print("OK: navigate_to_pose action server is available")
    return 0


if __name__ == "__main__":
    sys.exit(main())
