#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float32


class PicarxSafetyNode(Node):
    """Safety and command-gating node without direct hardware access."""

    def __init__(self) -> None:
        super().__init__('picarx_safety_node')

        self.declare_parameter('stop_distance_m', 0.2)
        self.declare_parameter('distance_timeout_sec', 1.0)
        self.declare_parameter('cmd_timeout_sec', 0.6)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self._distance_timeout_sec = float(self.get_parameter('distance_timeout_sec').value)
        self._cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._latest_distance: Optional[float] = None
        self._last_distance_time = self.get_clock().now()

        self._latest_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = False

        self._cmd_pub = self.create_publisher(Twist, '/picarx/cmd_vel', 10)
        self._stop_pub = self.create_publisher(Empty, '/picarx/stop', 10)

        self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd_vel_raw, 10)
        self.create_subscription(Float32, '/picarx/distance', self._on_distance, 10)
        self.create_subscription(Empty, '/picarx/stop_request', self._on_stop_request, 10)

        tick_period = 1.0 / max(publish_rate_hz, 1.0)
        self.create_timer(tick_period, self._publish_safe_command)

        self.get_logger().info('PI-CAR-X safety node started (no hardware access)')

    def _on_cmd_vel_raw(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = True

    def _on_distance(self, msg: Float32) -> None:
        self._latest_distance = float(msg.data)
        self._last_distance_time = self.get_clock().now()

    def _on_stop_request(self, _msg: Empty) -> None:
        self._stop_pub.publish(Empty())
        self._has_cmd = False

    def _distance_is_fresh(self) -> bool:
        age = (self.get_clock().now() - self._last_distance_time).nanoseconds / 1e9
        return age <= self._distance_timeout_sec

    def _cmd_is_fresh(self) -> bool:
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        return age <= self._cmd_timeout_sec

    def _publish_safe_command(self) -> None:
        if not self._has_cmd:
            return

        cmd = Twist()
        cmd.linear.x = float(self._latest_cmd.linear.x)
        cmd.angular.z = float(self._latest_cmd.angular.z)

        if not self._cmd_is_fresh():
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Block forward movement when obstacle is too close.
        if self._distance_is_fresh() and self._latest_distance is not None:
            if self._latest_distance < self._stop_distance_m and cmd.linear.x > 0.0:
                cmd.linear.x = 0.0
                if math.fabs(cmd.angular.z) < 1e-6:
                    cmd.angular.z = 0.0

        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PicarxSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
