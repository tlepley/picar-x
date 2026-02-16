#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class ExampleRunnerNode(Node):
    """Launches one PI-CAR-X example script as a managed subprocess."""

    def __init__(self) -> None:
        super().__init__('picarx_example_runner')

        self.declare_parameter('example_script', '4.avoiding_obstacles.py')
        self.declare_parameter('python_executable', sys.executable)
        self.declare_parameter('examples_dir', '')
        self.declare_parameter('working_dir', '')
        self.declare_parameter('pass_ros_args', False)

        example_script = str(self.get_parameter('example_script').value)
        python_exec = str(self.get_parameter('python_executable').value)
        pass_ros_args = bool(self.get_parameter('pass_ros_args').value)

        examples_dir_param = str(self.get_parameter('examples_dir').value)
        if examples_dir_param:
            examples_dir = Path(os.path.expanduser(examples_dir_param)).resolve()
        else:
            share_dir = Path(get_package_share_directory('picarx_remote_ros2')).resolve()
            examples_dir = share_dir / 'examples'

        script_path = (examples_dir / example_script).resolve()
        if not script_path.exists():
            raise FileNotFoundError(
                f'Example script not found: {script_path}. '
                f'Set parameter examples_dir or install package data files.'
            )

        working_dir_param = str(self.get_parameter('working_dir').value)
        if working_dir_param:
            working_dir = str(Path(os.path.expanduser(working_dir_param)).resolve())
        else:
            working_dir = str(script_path.parent)

        command = [python_exec, '-m', 'picarx_remote_ros2.remote_apps.remote_example_launcher', str(script_path)]
        if pass_ros_args:
            command.extend(sys.argv[1:])

        self.get_logger().info(f'Starting PI-CAR-X example: {script_path.name}')
        self.get_logger().info(f'Command: {command}')

        self._proc = subprocess.Popen(
            command,
            cwd=working_dir,
            stdin=None,
            stdout=None,
            stderr=None,
            start_new_session=True,
        )

        self._check_timer = self.create_timer(0.5, self._check_child)

    def _check_child(self) -> None:
        if self._proc is None:
            return

        ret = self._proc.poll()
        if ret is None:
            return

        if ret == 0:
            self.get_logger().info('Example script exited successfully, shutting down ROS node.')
        else:
            self.get_logger().error(f'Example script exited with code {ret}, shutting down ROS node.')

        self._proc = None
        rclpy.shutdown()

    def destroy_node(self) -> bool:
        proc = getattr(self, '_proc', None)
        if proc is not None and proc.poll() is None:
            self.get_logger().info('Stopping running example process...')
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Example did not stop on SIGINT, sending SIGTERM.')
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning('Example did not stop on SIGTERM, sending SIGKILL.')
                    os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = ExampleRunnerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
