#!/usr/bin/env python3
"""Hardware abstraction layer for PI-CAR-X ROS nodes.

Only this adapter talks directly to the picarx.Picarx API.
"""

import math
import os
from typing import List

from picarx import Picarx


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class PicarxHardwareAdapter:
    """Single-owner hardware adapter around picarx.Picarx."""

    def __init__(
        self,
        *,
        max_speed: float,
        max_steering_deg: float,
        steering_gain_deg_per_rad_s: float,
        direction_servo_pin: str,
        config_path: str,
        ultrasonic_trig_pin: str,
        ultrasonic_echo_pin: str,
    ) -> None:
        self.max_speed = float(max_speed)
        self.max_steering_deg = float(max_steering_deg)
        self.steering_gain = float(steering_gain_deg_per_rad_s)
        self._config_path = os.path.expanduser(config_path)
        self._ultrasonic_trig_pin = ultrasonic_trig_pin
        self._ultrasonic_echo_pin = ultrasonic_echo_pin
        self._direction_servo_pin = direction_servo_pin

        os.makedirs(os.path.dirname(self._config_path), exist_ok=True)
        self._px = Picarx(
            servo_pins=['P0', 'P1', self._direction_servo_pin],
            config=self._config_path,
            ultrasonic_pins=[self._ultrasonic_trig_pin, self._ultrasonic_echo_pin],
        )
        self._current_speed = 0.0
        self._px.stop()

    @property
    def current_speed(self) -> float:
        return self._current_speed

    def stop(self) -> None:
        self._current_speed = 0.0
        self._px.stop()

    def set_camera_pan(self, angle_deg: float) -> None:
        self._px.set_cam_pan_angle(float(angle_deg))

    def set_camera_tilt(self, angle_deg: float) -> None:
        self._px.set_cam_tilt_angle(float(angle_deg))

    def apply_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        steering = clamp(float(angular_z) * self.steering_gain, -self.max_steering_deg, self.max_steering_deg)
        self._px.set_dir_servo_angle(steering)
        self.set_speed(float(linear_x))

    def set_speed(self, speed: float) -> None:
        speed = clamp(float(speed), -self.max_speed, self.max_speed)
        self._current_speed = speed

        if math.isclose(speed, 0.0, abs_tol=1e-6):
            self._px.stop()
            return

        if speed > 0:
            self._px.forward(int(speed))
        else:
            self._px.backward(int(abs(speed)))

    def set_steering(self, steering_deg: float) -> None:
        steering = clamp(float(steering_deg), -self.max_steering_deg, self.max_steering_deg)
        self._px.set_dir_servo_angle(steering)

    def get_distance(self) -> float:
        return float(self._px.get_distance())

    def get_grayscale(self) -> List[float]:
        return [float(v) for v in self._px.get_grayscale_data()]

    def set_servo_offsets(self, dir_offset: float, cam_pan_offset: float, cam_tilt_offset: float) -> None:
        self._px.dir_servo_calibrate(float(dir_offset))
        self._px.cam_pan_servo_calibrate(float(cam_pan_offset))
        self._px.cam_tilt_servo_calibrate(float(cam_tilt_offset))

    def set_motor_directions(self, left: int, right: int) -> None:
        left = 1 if int(left) >= 0 else -1
        right = 1 if int(right) >= 0 else -1
        self._px.motor_direction_calibrate(1, left)
        self._px.motor_direction_calibrate(2, right)

    def save_calibration(self) -> None:
        # Calibration setters persist immediately through picarx fileDB.
        return

    def load_calibration(self) -> None:
        self._reconnect_hardware()

    def reset_calibration(self) -> None:
        self.set_servo_offsets(0.0, 0.0, 0.0)
        self.set_motor_directions(1, 1)

    def get_calibration_state(self) -> List[float]:
        return [
            float(self._px.dir_cali_val),
            float(self._px.cam_pan_cali_val),
            float(self._px.cam_tilt_cali_val),
            float(self._px.cali_dir_value[0]),
            float(self._px.cali_dir_value[1]),
        ]

    def _reconnect_hardware(self) -> None:
        self.stop()
        self._px = Picarx(
            servo_pins=['P0', 'P1', self._direction_servo_pin],
            config=self._config_path,
            ultrasonic_pins=[self._ultrasonic_trig_pin, self._ultrasonic_echo_pin],
        )
