#!/usr/bin/env python3
"""Interactive local camera control CLI for the PI-CAR-X camera publisher."""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class CameraControlClient(Node):
    """Small interactive helper for toggling the local camera stream."""

    def __init__(self) -> None:
        super().__init__('picarx_local_camera_control_cli')
        self._start_cli = self.create_client(Trigger, '/picarx_camera_publisher_node/start')
        self._stop_cli = self.create_client(Trigger, '/picarx_camera_publisher_node/stop')

    def call_trigger(self, action: str) -> bool:
        client = {'start': self._start_cli, 'stop': self._stop_cli}[action]
        service_name = f'/picarx_camera_publisher_node/{action}'
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service unavailable: {service_name}')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(f'Service call failed: {service_name}')
            return False

        result = future.result()
        log = self.get_logger().info if result.success else self.get_logger().error
        log(f'{service_name}: {result.message}')
        return bool(result.success)


def read_key() -> str:
    """Read one key from stdin in raw mode."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], None)
        if ready:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraControlClient()
    ok = True

    print('Local camera control')
    print('  s: start camera stream')
    print('  x: stop camera stream')
    print('  q or Esc: quit')

    try:
        while rclpy.ok():
            key = read_key()
            if key in ('q', '\x1b'):
                print('\nExiting camera control.')
                break
            if key == 's':
                ok = node.call_trigger('start') and ok
                continue
            if key == 'x':
                ok = node.call_trigger('stop') and ok
                continue
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
