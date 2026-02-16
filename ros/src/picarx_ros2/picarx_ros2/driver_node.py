#!/usr/bin/env python3
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Float32MultiArray

from .hardware import PicarxHardwareAdapter


class PicarxDriverNode(Node):
    """Only node that owns and accesses the PI-CAR-X hardware."""

    def __init__(self) -> None:
        super().__init__('picarx_driver_node')
        default_config_path = os.path.expanduser('~/.config/picar-x/picar-x.conf')

        self.declare_parameter('max_speed', 100.0)
        self.declare_parameter('max_steering_deg', 30.0)
        self.declare_parameter('steering_gain_deg_per_rad_s', 30.0)
        self.declare_parameter('cmd_timeout_sec', 0.6)
        self.declare_parameter('sensor_rate_hz', 10.0)
        self.declare_parameter('config_path', default_config_path)
        self.declare_parameter('ultrasonic_trig_pin', 'D0')
        self.declare_parameter('ultrasonic_echo_pin', 'D1')

        max_speed = float(self.get_parameter('max_speed').value)
        max_steering_deg = float(self.get_parameter('max_steering_deg').value)
        steering_gain = float(self.get_parameter('steering_gain_deg_per_rad_s').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        sensor_rate_hz = float(self.get_parameter('sensor_rate_hz').value)
        config_path = str(self.get_parameter('config_path').value)
        ultrasonic_trig_pin = str(self.get_parameter('ultrasonic_trig_pin').value)
        ultrasonic_echo_pin = str(self.get_parameter('ultrasonic_echo_pin').value)

        self._hardware = PicarxHardwareAdapter(
            max_speed=max_speed,
            max_steering_deg=max_steering_deg,
            steering_gain_deg_per_rad_s=steering_gain,
            config_path=config_path,
            ultrasonic_trig_pin=ultrasonic_trig_pin,
            ultrasonic_echo_pin=ultrasonic_echo_pin,
        )

        self._last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, '/picarx/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Empty, '/picarx/stop', self._on_stop, 10)
        self.create_subscription(Float32, '/picarx/speed', self._on_speed, 10)
        self.create_subscription(Float32, '/picarx/steering', self._on_steering, 10)
        self.create_subscription(Float32, '/picarx/camera_pan', self._on_camera_pan, 10)
        self.create_subscription(Float32, '/picarx/camera_tilt', self._on_camera_tilt, 10)

        self._distance_pub = self.create_publisher(Float32, '/picarx/distance', 10)
        self._grayscale_pub = self.create_publisher(Float32MultiArray, '/picarx/grayscale', 10)

        sensor_period = 1.0 / max(sensor_rate_hz, 0.1)
        self.create_timer(sensor_period, self._publish_sensors)
        self.create_timer(0.05, self._timeout_tick)

        self.get_logger().info('PI-CAR-X driver node started (single hardware owner)')

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._hardware.apply_cmd_vel(msg.linear.x, msg.angular.z)

    def _on_stop(self, _msg: Empty) -> None:
        self._hardware.stop()

    def _on_speed(self, msg: Float32) -> None:
        self._last_cmd_time = self.get_clock().now()
        self._hardware.set_speed(msg.data)

    def _on_steering(self, msg: Float32) -> None:
        self._hardware.set_steering(msg.data)

    def _on_camera_pan(self, msg: Float32) -> None:
        self._hardware.set_camera_pan(msg.data)

    def _on_camera_tilt(self, msg: Float32) -> None:
        self._hardware.set_camera_tilt(msg.data)

    def _publish_sensors(self) -> None:
        distance_msg = Float32()
        distance_msg.data = self._hardware.get_distance()
        self._distance_pub.publish(distance_msg)

        grayscale_msg = Float32MultiArray()
        grayscale_msg.data = self._hardware.get_grayscale()
        self._grayscale_pub.publish(grayscale_msg)

    def _timeout_tick(self) -> None:
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self._cmd_timeout and not math.isclose(self._hardware.current_speed, 0.0, abs_tol=1e-6):
            self._hardware.stop()

    def destroy_node(self) -> bool:
        try:
            self._hardware.stop()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PicarxDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
