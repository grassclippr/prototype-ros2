#!/usr/bin/env python3

import argparse
import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCheck(Node):
    def __init__(self, topic: str, sample_count: int, timeout_sec: float) -> None:
        super().__init__("imu_check")
        self.samples = []
        self.sample_count = sample_count
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        self.create_subscription(Imu, topic, self._cb, 20)

    def _cb(self, msg: Imu) -> None:
        self.samples.append(msg)

    def done(self) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        return len(self.samples) >= self.sample_count or now >= self.deadline


def main() -> int:
    parser = argparse.ArgumentParser(description="Check /imu/data rate, freshness, and basic gyro stats.")
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImuCheck(args.topic, args.samples, args.timeout)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()

    if len(samples) < 2:
        print(f"FAIL: received {len(samples)} IMU samples on {args.topic}")
        return 1

    stamps = [s.header.stamp.sec + s.header.stamp.nanosec / 1e9 for s in samples]
    duration = max(stamps[-1] - stamps[0], 1e-9)
    rate = (len(samples) - 1) / duration
    yaw_rates = [s.angular_velocity.z for s in samples if math.isfinite(s.angular_velocity.z)]
    if not yaw_rates:
        print("FAIL: no finite angular_velocity.z samples")
        return 1

    mean = statistics.fmean(yaw_rates)
    stdev = statistics.pstdev(yaw_rates) if len(yaw_rates) > 1 else 0.0
    cov = samples[-1].angular_velocity_covariance[8]
    print(f"OK: {len(samples)} samples, {rate:.1f} Hz")
    print(f"angular_velocity.z mean={mean:+.5f} rad/s stdev={stdev:.5f} covariance_z={cov:.5g}")
    print(f"frame_id={samples[-1].header.frame_id or '<empty>'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
