#!/usr/bin/env python3
"""Remote stare-at-you app using the ROS image stream."""

from __future__ import annotations

from time import sleep

import cv2

from ..remote_lib.remote_camera_stream import RemoteCameraStream
from ..remote_lib.remote_picarx import RemotePicarx
from ..remote_lib.vision_utils import build_face_detector, detect_face, draw_detection, ensure_display_available


def clamp_number(num, a, b):
    return max(min(num, max(a, b)), min(a, b))


def main() -> None:
    ensure_display_available('run_8_stare_at_you')
    stream = RemoteCameraStream()
    px = RemotePicarx()
    face_detector = build_face_detector()
    x_angle = 0.0
    y_angle = 0.0
    if not stream.request_start():
        raise SystemExit('Could not start remote camera stream')
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    try:
        while True:
            frame = stream.latest_frame()
            if frame is None:
                sleep(0.05)
                continue
            detection = detect_face(frame, face_detector)
            annotated = frame.copy()
            if detection is not None:
                draw_detection(annotated, detection, (0, 200, 0))
                coordinate_x, coordinate_y = detection.center
                x_angle += (coordinate_x * 10 / frame.shape[1]) - 5
                x_angle = clamp_number(x_angle, -35, 35)
                px.set_cam_pan_angle(x_angle)
                y_angle -= (coordinate_y * 10 / frame.shape[0]) - 5
                y_angle = clamp_number(y_angle, -35, 35)
                px.set_cam_tilt_angle(y_angle)
            cv2.imshow('PI-CAR-X Stare At You', annotated)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break
            sleep(0.05)
    finally:
        px.stop()
        stream.request_stop()
        stream.close()
        cv2.destroyAllWindows()
