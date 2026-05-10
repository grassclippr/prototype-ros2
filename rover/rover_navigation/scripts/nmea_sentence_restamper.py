#!/usr/bin/env python3

import rclpy
from nmea_msgs.msg import Sentence
from rclpy.node import Node


class NmeaSentenceRestamper(Node):
    def __init__(self) -> None:
        super().__init__("nmea_sentence_restamper")

        self.declare_parameter("input_topic", "/baseboard/nmea_sentence_raw")
        self.declare_parameter("output_topic", "/nmea_sentence")
        self.declare_parameter("default_frame_id", "gps")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._default_frame_id = self.get_parameter("default_frame_id").value

        self._publisher = self.create_publisher(Sentence, output_topic, 10)
        self.create_subscription(Sentence, input_topic, self._handle_sentence, 10)

    def _handle_sentence(self, msg: Sentence) -> None:
        restamped = Sentence()
        restamped.header.stamp = self.get_clock().now().to_msg()
        restamped.header.frame_id = msg.header.frame_id or self._default_frame_id
        restamped.sentence = msg.sentence
        self._publisher.publish(restamped)


def main() -> None:
    rclpy.init()
    node = NmeaSentenceRestamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
