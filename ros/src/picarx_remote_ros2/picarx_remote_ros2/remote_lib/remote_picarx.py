#!/usr/bin/env python3
"""Remote PI-CAR-X compatibility layer over ROS topics.

This module provides a Picarx-like API without direct GPIO access.
"""

from __future__ import annotations

import atexit
import threading
import time
from typing import Dict, List

import rclpy
from geometry_msgs import msg as ros_geometry_msg
from rclpy.node import Node
from std_msgs import msg as ros_msg


class _RemoteRosContext:
    """Shared ROS context for all RemotePicarx instances in one process."""

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)

        # In ROS, this process needs a Node to communicate on the graph.
        # This node runs on the remote/control machine, not on the car.
        self.node = Node(f'picarx_remote_client_{int(time.time() * 1000) % 100000}')
        self._lock = threading.Lock()
        self._distance = -1.0
        self._grayscale = [0.0, 0.0, 0.0]
        self._running = True

        # These publishers send driving commands onto ROS topics.
        # The local node running on the PI-CAR-X subscribes to them and
        # translates them into real hardware actions.
        self.cmd_pub = self.node.create_publisher(ros_geometry_msg.Twist, '/cmd_vel_raw', 10)
        self.stop_pub = self.node.create_publisher(ros_msg.Empty, '/picarx/stop_request', 10)
        self.speed_pub = self.node.create_publisher(ros_msg.Float32, '/picarx/speed', 10)
        self.steering_pub = self.node.create_publisher(ros_msg.Float32, '/picarx/steering', 10)
        self.cam_pan_pub = self.node.create_publisher(ros_msg.Float32, '/picarx/camera_pan', 10)
        self.cam_tilt_pub = self.node.create_publisher(ros_msg.Float32, '/picarx/camera_tilt', 10)

        # These subscriptions receive sensor values published by the car.
        # That lets remote code read distance / grayscale as if it were local.
        self.node.create_subscription(ros_msg.Float32, '/picarx/distance', self._on_distance, 10)
        self.node.create_subscription(ros_msg.Float32MultiArray, '/picarx/grayscale', self._on_grayscale, 10)

        # ROS callbacks only run while the node is being "spun".
        # We keep a small background thread alive so incoming sensor messages
        # update our cached state continuously.
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        atexit.register(self.close)

    def _on_distance(self, msg: ros_msg.Float32) -> None:
        with self._lock:
            self._distance = float(msg.data)

    def _on_grayscale(self, msg: ros_msg.Float32MultiArray) -> None:
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

    def get_grayscale(self) -> List[float]:
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


_CTX: _RemoteRosContext | None = None
_CTX_LOCK = threading.Lock()


def _get_ctx() -> _RemoteRosContext:
    global _CTX
    with _CTX_LOCK:
        if _CTX is None:
            _CTX = _RemoteRosContext()
    return _CTX


class _UltrasonicProxy:
    def __init__(self, owner: 'RemotePicarx') -> None:
        self._owner = owner

    def read(self) -> float:
        # Preserve the original picarx.ultrasonic.read() shape expected by
        # existing example scripts, but source the value from ROS.
        return self._owner.get_distance()

    def close(self) -> None:
        return


class RemotePicarx:
    """Drop-in subset of picarx.Picarx driven over ROS topics."""

    CONFIG = '/tmp/picar-x-remote.conf'
    DEFAULT_LINE_REF = [1000.0, 1000.0, 1000.0]
    DEFAULT_CLIFF_REF = [500.0, 500.0, 500.0]
    DIR_MIN = -30.0
    DIR_MAX = 30.0
    CAM_PAN_MIN = -90.0
    CAM_PAN_MAX = 90.0
    CAM_TILT_MIN = -35.0
    CAM_TILT_MAX = 65.0

    def __init__(
        self,
        servo_pins: List[str] | None = None,
        motor_pins: List[str] | None = None,
        grayscale_pins: List[str] | None = None,
        ultrasonic_pins: List[str] | None = None,
        config: str = CONFIG,
    ) -> None:
        # Keep the constructor compatible with the hardware Picarx API even
        # though the remote implementation does not use local pins at all.
        del servo_pins, motor_pins, grayscale_pins, ultrasonic_pins, config
        self._ctx = _get_ctx()

        self.dir_cali_val = 0.0
        self.cam_pan_cali_val = 0.0
        self.cam_tilt_cali_val = 0.0
        self.cali_dir_value = [1, 1]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0.0
        self.line_reference = list(self.DEFAULT_LINE_REF)
        self.cliff_reference = list(self.DEFAULT_CLIFF_REF)
        self._last_motor: Dict[int, float] = {1: 0.0, 2: 0.0}
        self.ultrasonic = _UltrasonicProxy(self)

        # Start from a safe state when a remote client connects.
        self.stop()

    def set_motor_speed(self, motor: int, speed: float) -> None:
        motor = 1 if int(motor) != 2 else 2
        self._last_motor[motor] = float(speed)
        left = self._last_motor[1]
        right = self._last_motor[2]
        # The original API exposes left/right wheel speed, but the ROS side is
        # modeled as speed + steering. This converts one abstraction into the
        # other as a best-effort approximation.
        linear = (left - right) / 2.0
        steering = max(self.DIR_MIN, min(self.DIR_MAX, (left + right) / 2.0))
        self.set_dir_servo_angle(steering)
        self._publish_speed(linear)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value

    def motor_direction_calibrate(self, motor, value):
        idx = max(0, min(1, int(motor) - 1))
        self.cali_dir_value[idx] = int(value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = float(value)

    def set_dir_servo_angle(self, value):
        steering = max(self.DIR_MIN, min(self.DIR_MAX, float(value)))
        self.dir_current_angle = steering
        msg = ros_msg.Float32()
        msg.data = steering
        # Publish steering intent; the local driver node applies it to the
        # actual steering servo on the car.
        self._ctx.steering_pub.publish(msg)

    def set_dir_servo_pin(self, pin):
        del pin

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = float(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = float(value)

    def set_cam_pan_angle(self, value):
        pan = max(self.CAM_PAN_MIN, min(self.CAM_PAN_MAX, float(value)))
        msg = ros_msg.Float32()
        msg.data = pan
        self._ctx.cam_pan_pub.publish(msg)

    def set_cam_tilt_angle(self, value):
        tilt = max(self.CAM_TILT_MIN, min(self.CAM_TILT_MAX, float(value)))
        msg = ros_msg.Float32()
        msg.data = tilt
        self._ctx.cam_tilt_pub.publish(msg)

    def set_power(self, speed):
        self._publish_speed(float(speed))

    def backward(self, speed):
        self._publish_speed(-abs(float(speed)))

    def forward(self, speed):
        self._publish_speed(abs(float(speed)))

    def stop(self):
        self._publish_speed(0.0)
        # Stop is split in two signals:
        # - speed=0 requests no forward/backward motion
        # - stop_request lets the safety/local side force an immediate halt
        self._ctx.stop_pub.publish(ros_msg.Empty())

    def _publish_speed(self, speed: float) -> None:
        msg = ros_msg.Float32()
        msg.data = float(speed)
        # This topic carries a simple scalar speed command from the remote
        # client to the PI-CAR-X driver node.
        self._ctx.speed_pub.publish(msg)

    def get_distance(self):
        # Return the latest value received asynchronously from ROS.
        return self._ctx.get_distance()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = [float(v) for v in value]
            return
        raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        # Return the latest line sensor values received from the car.
        return self._ctx.get_grayscale()

    def get_line_status(self, gm_val_list):
        status = []
        for idx in range(3):
            is_background = 1 if float(gm_val_list[idx]) > float(self.line_reference[idx]) else 0
            status.append(is_background)
        return status

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self, gm_val_list):
        for idx in range(3):
            if float(gm_val_list[idx]) <= float(self.cliff_reference[idx]):
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = [float(v) for v in value]
            return
        raise ValueError("cliff reference must be a 1*3 list")

    def reset(self):
        # Restore a neutral state remotely; the local hardware node performs
        # the actual servo and motor movements after receiving these topics.
        self.stop()
        self.set_dir_servo_angle(0.0)
        self.set_cam_tilt_angle(0.0)
        self.set_cam_pan_angle(0.0)

    def close(self):
        self.reset()
        self.ultrasonic.close()
