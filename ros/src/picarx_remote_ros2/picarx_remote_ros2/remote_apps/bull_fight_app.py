#!/usr/bin/env python3
"""Remote bull-fight app using color detection from the ROS image stream."""

from __future__ import annotations

from time import sleep

import cv2

from ..remote_lib.remote_camera_stream import RemoteCameraStream
from ..remote_lib.remote_picarx import RemotePicarx
from ..remote_lib.vision_utils import detect_color, draw_detection, ensure_display_available


def clamp_number(num, a, b):
    return max(min(num, max(a, b)), min(a, b))


def main() -> None:
    ensure_display_available('run_10_bull_fight')
    stream = RemoteCameraStream()
    px = RemotePicarx()
    speed = 50
    x_angle = 0.0
    y_angle = 0.0
    if not stream.request_start():
        raise SystemExit('Could not start remote camera stream')
    try:
        while True:
            frame = stream.latest_frame()
            if frame is None:
                sleep(0.05)
                continue
            detection = detect_color(frame, 'red')
            annotated = frame.copy()
            if detection is not None:
                draw_detection(annotated, detection, (0, 255, 255))
                coordinate_x, coordinate_y = detection.center
                x_angle += (coordinate_x * 10 / frame.shape[1]) - 5
                x_angle = clamp_number(x_angle, -35, 35)
                px.set_cam_pan_angle(x_angle)
                y_angle -= (coordinate_y * 10 / frame.shape[0]) - 5
                y_angle = clamp_number(y_angle, -35, 35)
                px.set_cam_tilt_angle(y_angle)
                px.set_dir_servo_angle(x_angle)
                px.forward(speed)
            else:
                px.stop()
            cv2.imshow('PI-CAR-X Bull Fight', annotated)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break
            sleep(0.05)
    finally:
        px.stop()
        stream.request_stop()
        stream.close()
        cv2.destroyAllWindows()
