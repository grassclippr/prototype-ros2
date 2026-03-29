#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from rover_msgs.msg import DriveCommand, DriveCommandAck

STATUS_RECEIVED = 0
STATUS_ACCEPTED = 1
STATUS_REJECTED = 2
STATUS_TIMED_OUT = 3


class DriveCommandSender(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("drive_command_sender")
        self._seq = int(args.seq)
        self._ack_received = False
        self._ack_status = None
        self._ack_detail = ""

        self._publisher = self.create_publisher(DriveCommand, args.command_topic, 10)
        self._subscription = self.create_subscription(
            DriveCommandAck,
            args.ack_topic,
            self._handle_ack,
            10,
        )

        self._timer = self.create_timer(0.1, self._publish_once)
        self._sent = False
        self._command = DriveCommand()
        self._command.stamp = self.get_clock().now().to_msg()
        self._command.seq = self._seq
        self._command.linear_x = float(args.linear_x)
        self._command.angular_z = float(args.angular_z)
        self._command.timeout_ms = int(args.timeout_ms)

    def _publish_once(self) -> None:
        if self._ack_received:
            return
        self._publisher.publish(self._command)
        if not self._sent:
            self.get_logger().info(
                f"sent seq={self._command.seq} linear_x={self._command.linear_x:.3f} angular_z={self._command.angular_z:.3f}"
            )
            self._sent = True

    def _handle_ack(self, ack: DriveCommandAck) -> None:
        if ack.seq != self._seq:
            return
        self._ack_received = True
        self._ack_status = int(ack.status)
        self._ack_detail = ack.detail


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Publish one DriveCommand and wait for the matching ack.")
    parser.add_argument("--linear-x", type=float, required=True)
    parser.add_argument("--angular-z", type=float, required=True)
    parser.add_argument("--timeout-ms", type=int, default=500)
    parser.add_argument("--seq", type=int, default=1)
    parser.add_argument("--command-topic", default="/drive_command")
    parser.add_argument("--ack-topic", default="/drive_command_ack")
    parser.add_argument("--wait-seconds", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = DriveCommandSender(args)
    deadline = node.get_clock().now().nanoseconds + int(args.wait_seconds * 1e9)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._ack_received:
                status_name = {
                    STATUS_RECEIVED: "RECEIVED",
                    STATUS_ACCEPTED: "ACCEPTED",
                    STATUS_REJECTED: "REJECTED",
                    STATUS_TIMED_OUT: "TIMED_OUT",
                }.get(node._ack_status, f"UNKNOWN({node._ack_status})")
                print(f"ack seq={args.seq} status={status_name} detail={node._ack_detail}")
                return 0 if node._ack_status == STATUS_ACCEPTED else 2
            if node.get_clock().now().nanoseconds >= deadline:
                print(f"timeout waiting for ack seq={args.seq}")
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
