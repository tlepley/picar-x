#!/usr/bin/env python3
"""Helpers to consume the PI-CAR-X ROS camera stream on the remote side."""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class _RemoteCameraNode(Node):
    def __init__(self, image_topic: str, start_service: str, stop_service: str) -> None:
        super().__init__('picarx_remote_camera_stream')
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._start_client = self.create_client(Trigger, start_service)
        self._stop_client = self.create_client(Trigger, stop_service)
        self.create_subscription(Image, image_topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        frame = decode_image(msg)
        if frame is None:
            return
        with self._frame_lock:
            self._latest_frame = frame

    def latest_frame(self) -> Optional[np.ndarray]:
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()


def decode_image(msg: Image) -> Optional[np.ndarray]:
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(msg.encoding)
    if channels is None:
        return None

    frame = np.frombuffer(msg.data, dtype=np.uint8)
    expected_size = int(msg.height) * int(msg.width) * channels
    if frame.size != expected_size:
        return None

    if channels == 1:
        return frame.reshape((msg.height, msg.width))

    frame = frame.reshape((msg.height, msg.width, channels))
    if msg.encoding == 'rgb8':
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


class RemoteCameraStream:
    """Owns a background ROS executor and exposes the latest camera frame."""

    def __init__(
        self,
        image_topic: str = '/picarx/camera/image_raw',
        start_service: str = '/picarx_camera_publisher_node/start',
        stop_service: str = '/picarx_camera_publisher_node/stop',
    ) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = _RemoteCameraNode(image_topic, start_service, stop_service)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.05)

    def latest_frame(self) -> Optional[np.ndarray]:
        return self._node.latest_frame()

    def request_start(self) -> bool:
        return self._call_trigger(self._node._start_client)

    def request_stop(self) -> bool:
        return self._call_trigger(self._node._stop_client)

    def _call_trigger(self, client) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            return False
        future = client.call_async(Trigger.Request())
        done = threading.Event()

        def _mark_done(_future) -> None:
            done.set()

        future.add_done_callback(_mark_done)
        if not done.wait(timeout=5.0):
            return False
        if future.result() is None:
            return False
        return bool(future.result().success)

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
