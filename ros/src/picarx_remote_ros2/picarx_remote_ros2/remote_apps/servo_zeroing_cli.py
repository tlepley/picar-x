#!/usr/bin/env python3
"""Remote one-shot client for PI-CAR-X raw servo zeroing."""

from __future__ import annotations

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServoZeroingClient(Node):
    def __init__(self) -> None:
        super().__init__('picarx_remote_servo_zeroing_cli')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._cli = self.create_client(Trigger, '/picarx/servo_zeroing')

    def run(self) -> bool:
        if not self._cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service unavailable: /picarx/servo_zeroing')
            return False
        future = self._cli.call_async(Trigger.Request())
        self._executor.spin_until_future_complete(future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('Service call failed: /picarx/servo_zeroing')
            return False
        result = future.result()
        log = self.get_logger().info if result.success else self.get_logger().error
        log(result.message)
        return bool(result.success)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoZeroingClient()
    ok = False
    try:
        ok = node.run()
    finally:
        node._executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
