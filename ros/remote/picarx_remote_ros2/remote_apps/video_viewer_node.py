#!/usr/bin/env python3
"""ROS 2 viewer for the PI-CAR-X camera stream on the remote side."""

from __future__ import annotations

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class PicarxVideoViewerNode(Node):
    """Displays the PI-CAR-X camera stream in an OpenCV window."""

    def __init__(self) -> None:
        super().__init__('picarx_video_viewer_node')

        self.declare_parameter('image_topic', '/picarx/camera/image_raw')
        self.declare_parameter('window_name', 'PI-CAR-X Camera')
        self.declare_parameter('display_width', 0)
        self.declare_parameter('display_height', 0)
        self.declare_parameter('start_camera_on_launch', True)
        self.declare_parameter('stop_camera_on_exit', True)
        self.declare_parameter('camera_start_service', '/picarx_camera_publisher_node/start')
        self.declare_parameter('camera_stop_service', '/picarx_camera_publisher_node/stop')

        image_topic = str(self.get_parameter('image_topic').value)
        self._window_name = str(self.get_parameter('window_name').value)
        self._display_width = int(self.get_parameter('display_width').value)
        self._display_height = int(self.get_parameter('display_height').value)
        self._start_camera_on_launch = bool(self.get_parameter('start_camera_on_launch').value)
        self._stop_camera_on_exit = bool(self.get_parameter('stop_camera_on_exit').value)
        self._camera_start_service = str(self.get_parameter('camera_start_service').value)
        self._camera_stop_service = str(self.get_parameter('camera_stop_service').value)
        self._headless = not bool(os.environ.get('DISPLAY'))
        self._warned_headless = False
        self._shutdown_requested = False
        self._start_client = self.create_client(Trigger, self._camera_start_service)
        self._stop_client = self.create_client(Trigger, self._camera_stop_service)

        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(f'Subscribed to {image_topic}')
        self.get_logger().info("Press Escape in the video window to stop the viewer and turn the camera stream off.")
        # The viewer owns the camera stream session by default: start on launch, stop on exit.
        if self._start_camera_on_launch:
            self._call_trigger(self._start_client, self._camera_start_service, action='start')

    def _on_image(self, msg: Image) -> None:
        # Convert the ROS image payload into an OpenCV frame before any display logic.
        frame = self._decode_image(msg)
        if frame is None:
            return

        if self._display_width > 0 and self._display_height > 0:
            frame = cv2.resize(frame, (self._display_width, self._display_height))

        if self._headless:
            if not self._warned_headless:
                self.get_logger().warning('DISPLAY is not set; video stream is received but cannot be shown in a window.')
                self._warned_headless = True
            return

        cv2.imshow(self._window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            # Escape requests a clean shutdown so destroy_node() can send camera stop.
            self.get_logger().info('Escape pressed. Stopping viewer and requesting camera stop.')
            self._shutdown_requested = True
            if rclpy.ok():
                rclpy.shutdown()

    def _decode_image(self, msg: Image):
        # Keep decoding intentionally narrow: this viewer is only meant for the encodings
        # currently published by the PI-CAR-X camera node.
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(msg.encoding)
        if channels is None:
            self.get_logger().warning(f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = int(msg.height) * int(msg.width) * channels
        if frame.size != expected_size:
            self.get_logger().warning(
                f'Unexpected image buffer size: expected {expected_size}, got {frame.size}',
                throttle_duration_sec=5.0,
            )
            return None

        if channels == 1:
            # mono8 arrives as a flat byte buffer; reshape it into a 2D grayscale image.
            return frame.reshape((msg.height, msg.width))

        # Color images arrive as an interleaved flat buffer and must be reshaped into HxWxC.
        frame = frame.reshape((msg.height, msg.width, channels))
        if msg.encoding == 'rgb8':
            # OpenCV display routines expect BGR ordering, not RGB.
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def destroy_node(self) -> bool:
        if self._stop_camera_on_exit:
            self._call_trigger(self._stop_client, self._camera_stop_service, action='stop')
        if not self._headless:
            cv2.destroyAllWindows()
        return super().destroy_node()

    def _call_trigger(self, client, service_name: str, *, action: str) -> None:
        # Service calls are synchronous here on purpose: start/stop camera state must be
        # explicit from the viewer's perspective.
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f'Camera {action} service unavailable: {service_name}',
                throttle_duration_sec=5.0,
            )
            return

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warning(
                f'Timed out waiting for camera {action} response from {service_name}',
                throttle_duration_sec=5.0,
            )
            return

        response = future.result()
        if response is None:
            self.get_logger().warning(
                f'Camera {action} request to {service_name} returned no response',
                throttle_duration_sec=5.0,
            )
            return

        log = self.get_logger().info if response.success else self.get_logger().warning
        log(f'Camera {action}: {response.message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = PicarxVideoViewerNode()
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
