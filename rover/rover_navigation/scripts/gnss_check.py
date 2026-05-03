#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GnssCheck(Node):
    def __init__(self, topic: str, sample_count: int, timeout_sec: float) -> None:
        super().__init__("gnss_check")
        self.samples = []
        self.sample_count = sample_count
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        self.create_subscription(NavSatFix, topic, self._cb, 10)

    def _cb(self, msg: NavSatFix) -> None:
        self.samples.append(msg)

    def done(self) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        return len(self.samples) >= self.sample_count or now >= self.deadline


def main() -> int:
    parser = argparse.ArgumentParser(description="Check GNSS NavSatFix presence and freshness.")
    parser.add_argument("--topic", default="/fix")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = GnssCheck(args.topic, args.samples, args.timeout)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()

    if not samples:
        print(f"FAIL: no NavSatFix samples on {args.topic}")
        return 1

    last = samples[-1]
    status = last.status.status
    cov = last.position_covariance
    print(f"OK: received {len(samples)} samples on {args.topic}")
    print(f"status={status} lat={last.latitude:.8f} lon={last.longitude:.8f} alt={last.altitude:.2f}")
    print(f"covariance_diag=[{cov[0]:.3g}, {cov[4]:.3g}, {cov[8]:.3g}] type={last.position_covariance_type}")
    if status < 0:
        print("WARN: GNSS reports no fix")
    return 0


if __name__ == "__main__":
    sys.exit(main())
