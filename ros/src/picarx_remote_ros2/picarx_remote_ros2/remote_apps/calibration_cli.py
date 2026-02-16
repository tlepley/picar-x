#!/usr/bin/env python3
"""Remote calibration CLI for PI-CAR-X local hardware node."""

from __future__ import annotations

import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger


class CalibrationClient(Node):
    def __init__(self) -> None:
        super().__init__('picarx_remote_calibration_cli')
        self._servo_pub = self.create_publisher(Float32MultiArray, '/picarx/calibration/servo_offsets', 10)
        self._motor_pub = self.create_publisher(Int32MultiArray, '/picarx/calibration/motor_directions', 10)

        self._save_cli = self.create_client(Trigger, '/picarx/calibration/save')
        self._load_cli = self.create_client(Trigger, '/picarx/calibration/load')
        self._reset_cli = self.create_client(Trigger, '/picarx/calibration/reset')
        self._get_cli = self.create_client(Trigger, '/picarx/calibration/get')

        self._last_state: Optional[list[float]] = None
        self.create_subscription(Float32MultiArray, '/picarx/calibration/state', self._on_state, 10)

    def _on_state(self, msg: Float32MultiArray) -> None:
        self._last_state = list(msg.data)

    def set_servo_offsets(self, dir_offset: float, pan_offset: float, tilt_offset: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(dir_offset), float(pan_offset), float(tilt_offset)]
        self._servo_pub.publish(msg)
        self.get_logger().info(f'Sent servo offsets: dir={dir_offset}, pan={pan_offset}, tilt={tilt_offset}')

    def set_motor_directions(self, left: int, right: int) -> None:
        msg = Int32MultiArray()
        msg.data = [1 if int(left) >= 0 else -1, 1 if int(right) >= 0 else -1]
        self._motor_pub.publish(msg)
        self.get_logger().info(f'Sent motor directions: left={msg.data[0]}, right={msg.data[1]}')

    def call_trigger(self, name: str) -> bool:
        cli = {
            'save': self._save_cli,
            'load': self._load_cli,
            'reset': self._reset_cli,
            'get': self._get_cli,
        }[name]
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service unavailable: /picarx/calibration/{name}')
            return False
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'Service call failed: /picarx/calibration/{name}')
            return False
        result = future.result()
        level = self.get_logger().info if result.success else self.get_logger().error
        level(f'/picarx/calibration/{name}: {result.message}')
        return bool(result.success)

    def print_state(self) -> bool:
        state_message = self.get_state_message()
        if state_message is None:
            return False
        print(f'calibration_state {state_message}')
        return True

    def get_state_message(self) -> Optional[str]:
        cli = self._get_cli
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service unavailable: /picarx/calibration/get')
            return None
        future = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Service call failed: /picarx/calibration/get')
            return None
        result = future.result()
        if not result.success:
            self.get_logger().error(result.message)
            return None
        return result.message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Remote calibration utility for PI-CAR-X.')
    parser.add_argument('--set-servo', nargs=3, type=float, metavar=('DIR', 'PAN', 'TILT'))
    parser.add_argument('--set-motor', nargs=2, type=int, metavar=('LEFT', 'RIGHT'))
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--load', action='store_true')
    parser.add_argument('--reset', action='store_true')
    parser.add_argument('--show', action='store_true')
    return parser.parse_args()


def main(args=None) -> None:
    parsed = parse_args()
    rclpy.init(args=args)
    node = CalibrationClient()

    ok = True
    try:
        if parsed.set_servo is not None:
            node.set_servo_offsets(*parsed.set_servo)
            rclpy.spin_once(node, timeout_sec=0.1)
        if parsed.set_motor is not None:
            node.set_motor_directions(*parsed.set_motor)
            rclpy.spin_once(node, timeout_sec=0.1)
        if parsed.reset:
            ok = node.call_trigger('reset') and ok
        if parsed.load:
            ok = node.call_trigger('load') and ok
        if parsed.save:
            ok = node.call_trigger('save') and ok
        if parsed.show:
            ok = node.print_state() and ok
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
