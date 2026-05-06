#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


LINEAR_STEP = 0.01
ANGULAR_STEP = 0.10
MAX_LINEAR = 0.08
MAX_ANGULAR = 0.40
PUBLISH_PERIOD_SEC = 0.1


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class KeyboardTwistTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_twist_teleop")
        self._publisher = self.create_publisher(TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self._linear = 0.0
        self._angular = 0.0
        self._timer = self.create_timer(PUBLISH_PERIOD_SEC, self._publish)

    def handle_key(self, key: str) -> bool:
        if key == "w":
            self._linear = clamp(self._linear + LINEAR_STEP, -MAX_LINEAR, MAX_LINEAR)
        elif key == "s":
            self._linear = clamp(self._linear - LINEAR_STEP, -MAX_LINEAR, MAX_LINEAR)
        elif key == "a":
            self._angular = clamp(self._angular + ANGULAR_STEP, -MAX_ANGULAR, MAX_ANGULAR)
        elif key == "d":
            self._angular = clamp(self._angular - ANGULAR_STEP, -MAX_ANGULAR, MAX_ANGULAR)
        elif key in ("x", " "):
            self._linear = 0.0
            self._angular = 0.0
        elif key == "q":
            self.stop()
            return False
        else:
            return True

        self._print_state()
        return True

    def _print_state(self) -> None:
        print(
            f"\rlinear.x={self._linear:+.2f} m/s  angular.z={self._angular:+.2f} rad/s    ",
            end="",
            flush=True,
        )

    def _publish(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = self._linear
        msg.twist.angular.z = self._angular
        self._publisher.publish(msg)

    def stop(self) -> None:
        self._linear = 0.0
        self._angular = 0.0
        self._publish()
        self._print_state()
        print()


def read_key(timeout_sec: float = 0.1) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return ""
    return sys.stdin.read(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTwistTeleop()

    print("Keyboard teleop -> /diff_drive_controller/cmd_vel")
    print("  w/s : increase/decrease linear speed")
    print("  a/d : increase/decrease angular speed")
    print("  x or space : stop")
    print("  q : quit")
    node._print_state()

    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)
    tty.setcbreak(stdin_fd)

    try:
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(PUBLISH_PERIOD_SEC)
            if key:
                running = node.handle_key(key)
    finally:
        node.stop()
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
