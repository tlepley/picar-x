#!/usr/bin/env python3
"""Shared ROS runtime for remote PI-CAR-X helpers."""

from __future__ import annotations

import atexit
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


class RemoteRuntime:
    """Owns a single ROS node per process for all remote helper APIs."""

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = Node(f'picarx_remote_client_{int(time.time() * 1000) % 100000}')
        self._lock = threading.Lock()
        self._running = True
        self._distance = -1.0
        self._grayscale = [0.0, 0.0, 0.0]

        self.speed_pub = self.node.create_publisher(Float32, '/picarx/speed', 10)
        self.steering_pub = self.node.create_publisher(Float32, '/picarx/steering', 10)
        self.cam_pan_pub = self.node.create_publisher(Float32, '/picarx/camera_pan', 10)
        self.cam_tilt_pub = self.node.create_publisher(Float32, '/picarx/camera_tilt', 10)
        self.line_ref_pub = self.node.create_publisher(Float32MultiArray, '/picarx/calibration/line_reference', 10)
        self.cliff_ref_pub = self.node.create_publisher(Float32MultiArray, '/picarx/calibration/cliff_reference', 10)
        self.led_pub = self.node.create_publisher(String, '/picarx/embodiment/led', 10)
        self.sound_pub = self.node.create_publisher(String, '/picarx/embodiment/sound', 10)
        self.speech_pub = self.node.create_publisher(String, '/picarx/embodiment/speech', 10)

        self.node.create_subscription(Float32, '/picarx/distance', self._on_distance, 10)
        self.node.create_subscription(Float32MultiArray, '/picarx/grayscale', self._on_grayscale, 10)

        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        atexit.register(self.close)

    def _on_distance(self, msg: Float32) -> None:
        with self._lock:
            self._distance = float(msg.data)

    def _on_grayscale(self, msg: Float32MultiArray) -> None:
        vals = list(msg.data)
        if len(vals) >= 3:
            with self._lock:
                self._grayscale = [float(vals[0]), float(vals[1]), float(vals[2])]

    def _spin_loop(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def get_distance(self) -> float:
        with self._lock:
            return float(self._distance)

    def get_grayscale(self) -> list[float]:
        with self._lock:
            return list(self._grayscale)

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass


_CTX: Optional[RemoteRuntime] = None
_CTX_LOCK = threading.Lock()


def get_runtime() -> RemoteRuntime:
    global _CTX
    with _CTX_LOCK:
        if _CTX is None:
            _CTX = RemoteRuntime()
    return _CTX
