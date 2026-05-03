#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from adafruit_extended_bus import ExtendedI2C
from adafruit_icm20x import ICM20948


class Icm20948Node(Node):
    def __init__(self) -> None:
        super().__init__("icm20948")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x69)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "/imu/data")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("angular_velocity_variance", 0.0025)
        self.declare_parameter("linear_acceleration_variance", 0.04)

        i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        address_param = self.get_parameter("address")
        if address_param.type_ == Parameter.Type.STRING:
            address = int(address_param.value, 0)
        elif address_param.type_ == Parameter.Type.INTEGER:
            address = int(address_param.value)
        else:
            raise ValueError("address must be an integer or string literal like 0x69")
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.angular_velocity_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )
        self.linear_acceleration_variance = float(
            self.get_parameter("linear_acceleration_variance").value
        )

        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        try:
            self.i2c = ExtendedI2C(i2c_bus)
            self.sensor = ICM20948(self.i2c, address=address)
        except (RuntimeError, ValueError, OSError) as exc:
            raise RuntimeError(
                f"failed to initialize ICM20948 on /dev/i2c-{i2c_bus} at {address:#04x}"
            ) from exc

        self.publisher = self.create_publisher(Imu, topic, 20)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_imu)
        self.last_error_ns = 0
        self.get_logger().info(
            f"publishing IMU data from /dev/i2c-{i2c_bus} address {address:#04x} to {topic}"
        )

    def publish_imu(self) -> None:
        try:
            accel_x, accel_y, accel_z = self.sensor.acceleration
            gyro_x, gyro_y, gyro_z = self.sensor.gyro
        except OSError as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_error_ns > 5_000_000_000:
                self.get_logger().error(f"ICM20948 read failed: {exc}")
                self.last_error_ns = now_ns
            return

        if not all(
            math.isfinite(value)
            for value in (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
        ):
            self.get_logger().error("ICM20948 returned non-finite data")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        msg.angular_velocity_covariance[0] = self.angular_velocity_variance
        msg.angular_velocity_covariance[4] = self.angular_velocity_variance
        msg.angular_velocity_covariance[8] = self.angular_velocity_variance

        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        msg.linear_acceleration_covariance[0] = self.linear_acceleration_variance
        msg.linear_acceleration_covariance[4] = self.linear_acceleration_variance
        msg.linear_acceleration_covariance[8] = self.linear_acceleration_variance

        self.publisher.publish(msg)


def main() -> int:
    rclpy.init()
    node = Icm20948Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
