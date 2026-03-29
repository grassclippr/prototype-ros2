#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rover_msgs.msg import DriveCommand


class CmdVelToDriveCommand(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_drive_command")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/drive_command")
        self.declare_parameter("timeout_ms", 500)
        self._input_topic = self.get_parameter("input_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._timeout_ms = int(self.get_parameter("timeout_ms").value)
        self._seq = 0

        self._publisher = self.create_publisher(DriveCommand, self._output_topic, 10)
        self._subscription = self.create_subscription(
            Twist,
            self._input_topic,
            self._handle_cmd_vel,
            10,
        )

    def _handle_cmd_vel(self, msg: Twist) -> None:
        command = DriveCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.seq = self._seq
        command.linear_x = float(msg.linear.x)
        command.angular_z = float(msg.angular.z)
        command.timeout_ms = self._timeout_ms
        self._publisher.publish(command)
        self._seq += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToDriveCommand()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
