#!/usr/bin/env python3
"""Remote PI-CAR-X compatibility layer over ROS topics.

This module provides a Picarx-like API without direct GPIO access.
"""

from __future__ import annotations

from typing import Dict, List

from std_msgs import msg as ros_msg

from .remote_runtime import get_runtime


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
        self._ctx = get_runtime()

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
            msg = ros_msg.Float32MultiArray()
            msg.data = list(self.line_reference)
            self._ctx.line_ref_pub.publish(msg)
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
            msg = ros_msg.Float32MultiArray()
            msg.data = list(self.cliff_reference)
            self._ctx.cliff_ref_pub.publish(msg)
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
