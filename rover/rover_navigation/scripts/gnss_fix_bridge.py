#!/usr/bin/env python3

from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rover_baseboard_msgs.msg import GnssFix
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference


class GnssFixBridge(Node):
    def __init__(self) -> None:
        super().__init__("gnss_fix_bridge")
        self.declare_parameter("input_topic", "/baseboard/gnss_fix")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("time_reference_topic", "/time_reference")
        self.declare_parameter("frame_id", "gps")
        self._frame_id = str(self.get_parameter("frame_id").value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        input_topic = str(self.get_parameter("input_topic").value)
        fix_topic = str(self.get_parameter("fix_topic").value)
        time_reference_topic = str(self.get_parameter("time_reference_topic").value)

        self._fix_pub = self.create_publisher(NavSatFix, fix_topic, sensor_qos)
        self._time_pub = self.create_publisher(TimeReference, time_reference_topic, sensor_qos)
        self.create_subscription(GnssFix, input_topic, self._on_fix, sensor_qos)

    def _on_fix(self, msg: GnssFix) -> None:
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.latitude = msg.latitude_deg
        fix.longitude = msg.longitude_deg
        fix.altitude = msg.altitude_m
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        fix.position_covariance = [0.0] * 9
        fix.status.status = self._map_fix_quality(msg.fix_quality)
        fix.status.service = NavSatStatus.SERVICE_GPS
        self._fix_pub.publish(fix)

        if msg.utc_valid:
            time_reference = TimeReference()
            time_reference.header = fix.header
            time_reference.source = "gnss"
            time_reference.time_ref = self._decode_utc(msg)
            self._time_pub.publish(time_reference)

    @staticmethod
    def _map_fix_quality(fix_quality: int) -> int:
        if fix_quality <= 0:
            return NavSatStatus.STATUS_NO_FIX
        if fix_quality == 2:
            return NavSatStatus.STATUS_SBAS_FIX
        if fix_quality >= 4:
            return NavSatStatus.STATUS_GBAS_FIX
        return NavSatStatus.STATUS_FIX

    def _decode_utc(self, msg: GnssFix):
        try:
            dt = datetime(
                msg.utc_year,
                msg.utc_month,
                msg.utc_day,
                msg.utc_hour,
                msg.utc_minute,
                int(msg.utc_second),
                int((msg.utc_second - int(msg.utc_second)) * 1_000_000),
                tzinfo=timezone.utc,
            )
        except ValueError:
            return self.get_clock().now().to_msg()

        timestamp = dt.timestamp()
        whole = int(timestamp)
        nanos = int((timestamp - whole) * 1_000_000_000)
        stamp = self.get_clock().now().to_msg()
        stamp.sec = whole
        stamp.nanosec = nanos
        return stamp


def main() -> None:
    rclpy.init()
    node = GnssFixBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
