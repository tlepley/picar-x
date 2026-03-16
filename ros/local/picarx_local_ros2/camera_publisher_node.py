#!/usr/bin/env python3
"""ROS 2 camera publisher for the PI-CAR-X Raspberry Pi side."""

from __future__ import annotations

from typing import Optional, Protocol

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def _parse_camera_source(value: str):
    value = str(value).strip()
    if value.lstrip("-").isdigit():
        return int(value)
    return value


class _FrameSource(Protocol):
    def read(self): ...
    def close(self) -> None: ...


class _OpenCvFrameSource:
    def __init__(self, source, *, width: int, height: int, fps: float) -> None:
        self._capture = cv2.VideoCapture(source)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f'Failed to open camera source with OpenCV: {source!r}')

        if width > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:
            self._capture.set(cv2.CAP_PROP_FPS, fps)

    def read(self):
        ok, frame = self._capture.read()
        return ok, frame

    def close(self) -> None:
        self._capture.release()


class _PiCamera2FrameSource:
    def __init__(self, *, width: int, height: int) -> None:
        from picamera2 import Picamera2  # type: ignore

        self._camera = Picamera2()
        config = self._camera.create_preview_configuration(
            main={'size': (width, height), 'format': 'RGB888'}
        )
        self._camera.configure(config)
        self._camera.start()

    def read(self):
        frame = self._camera.capture_array()
        if frame is None:
            return False, None
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return True, frame

    def close(self) -> None:
        self._camera.close()


class PicarxCameraPublisherNode(Node):
    """Publishes camera frames as sensor_msgs/Image."""

    def __init__(self) -> None:
        super().__init__('picarx_camera_publisher_node')

        self.declare_parameter('camera_source', '0')
        self.declare_parameter('image_topic', '/picarx/camera/image_raw')
        self.declare_parameter('frame_id', 'picarx_camera')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('camera_backend', 'auto')

        camera_source = _parse_camera_source(str(self.get_parameter('camera_source').value))
        image_topic = str(self.get_parameter('image_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        backend = str(self.get_parameter('camera_backend').value).strip().lower()

        self._pub = self.create_publisher(Image, image_topic, 10)
        self._capture, backend_name = self._open_capture(
            camera_source,
            backend=backend,
            width=width,
            height=height,
            fps=fps,
        )

        timer_period = 1.0 / publish_rate_hz
        self.create_timer(timer_period, self._publish_frame)
        self.get_logger().info(
            f'Publishing camera frames on {image_topic} from source {camera_source!r} using backend {backend_name}'
        )

    def _open_capture(self, source, *, backend: str, width: int, height: int, fps: float):
        backends = [backend]
        if backend == 'auto':
            backends = ['picamera2', 'opencv']

        last_error = None
        for backend_name in backends:
            try:
                if backend_name == 'picamera2':
                    return _PiCamera2FrameSource(width=width, height=height), backend_name
                if backend_name == 'opencv':
                    return _OpenCvFrameSource(source, width=width, height=height, fps=fps), backend_name
                raise RuntimeError(f'Unsupported camera backend: {backend_name}')
            except Exception as exc:
                last_error = exc
                self.get_logger().warning(
                    f'Camera backend {backend_name} unavailable: {exc}',
                    throttle_duration_sec=5.0,
                )

        raise RuntimeError(f'Failed to open camera source {source!r}: {last_error}')

    def _publish_frame(self) -> None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            self.get_logger().warning('Failed to read camera frame.', throttle_duration_sec=5.0)
            return

        if len(frame.shape) != 3 or frame.shape[2] != 3:
            self.get_logger().warning('Ignoring frame with unsupported shape.', throttle_duration_sec=5.0)
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        capture: Optional[_FrameSource] = getattr(self, '_capture', None)
        if capture is not None:
            capture.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = PicarxCameraPublisherNode()
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
