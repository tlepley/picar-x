#!/usr/bin/env python3
"""Shared remote vision utilities built on OpenCV frames from ROS."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

FACE_CASCADE_CANDIDATES = [
    # OpenCV wheels often expose this helper path.
    os.path.join(getattr(getattr(cv2, 'data', object()), 'haarcascades', ''), 'haarcascade_frontalface_default.xml'),
    '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
    '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
    '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
]

CAT_CASCADE_CANDIDATES = [
    os.path.join(getattr(getattr(cv2, 'data', object()), 'haarcascades', ''), 'haarcascade_frontalcatface.xml'),
    os.path.join(getattr(getattr(cv2, 'data', object()), 'haarcascades', ''), 'haarcascade_frontalcatface_extended.xml'),
    '/usr/share/opencv4/haarcascades/haarcascade_frontalcatface.xml',
    '/usr/share/opencv4/haarcascades/haarcascade_frontalcatface_extended.xml',
    '/usr/share/opencv/haarcascades/haarcascade_frontalcatface.xml',
    '/usr/share/opencv/haarcascades/haarcascade_frontalcatface_extended.xml',
    '/usr/local/share/opencv4/haarcascades/haarcascade_frontalcatface.xml',
    '/usr/local/share/opencv4/haarcascades/haarcascade_frontalcatface_extended.xml',
]

COLOR_RANGES = {
    'red': [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    'orange': [((10, 120, 70), (22, 255, 255))],
    'yellow': [((22, 120, 70), (35, 255, 255))],
    'green': [((35, 80, 50), (85, 255, 255))],
    'blue': [((90, 80, 50), (130, 255, 255))],
    'purple': [((130, 80, 50), (165, 255, 255))],
}


@dataclass
class BoxDetection:
    label: str
    x: int
    y: int
    w: int
    h: int

    @property
    def center(self) -> tuple[int, int]:
        return (self.x + self.w // 2, self.y + self.h // 2)


def build_face_detector():
    for path in FACE_CASCADE_CANDIDATES:
        if not path or not os.path.exists(path):
            continue
        detector = cv2.CascadeClassifier(path)
        if not detector.empty():
            return detector
    return None


def build_cat_detector():
    for path in CAT_CASCADE_CANDIDATES:
        if not path or not os.path.exists(path):
            continue
        detector = cv2.CascadeClassifier(path)
        if not detector.empty():
            return detector
    return None


def detect_face(frame: np.ndarray, face_detector) -> Optional[BoxDetection]:
    if face_detector is None:
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    if len(faces) == 0:
        return None
    x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
    return BoxDetection('face', int(x), int(y), int(w), int(h))


def detect_cat(frame: np.ndarray, cat_detector) -> Optional[BoxDetection]:
    if cat_detector is None:
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cats = cat_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3, minSize=(40, 40))
    if len(cats) == 0:
        return None
    x, y, w, h = max(cats, key=lambda c: c[2] * c[3])
    return BoxDetection('cat', int(x), int(y), int(w), int(h))


def detect_color(frame: np.ndarray, color_name: str) -> Optional[BoxDetection]:
    if color_name not in COLOR_RANGES:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in COLOR_RANGES[color_name]:
        partial = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = partial if mask is None else cv2.bitwise_or(mask, partial)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 200:
        return None
    x, y, w, h = cv2.boundingRect(contour)
    return BoxDetection(color_name, int(x), int(y), int(w), int(h))


def detect_qr(frame: np.ndarray, qr_detector) -> tuple[Optional[str], Optional[np.ndarray]]:
    text, points, _ = qr_detector.detectAndDecode(frame)
    return (text or None, points)


def draw_detection(frame: np.ndarray, detection: BoxDetection, color: tuple[int, int, int]) -> np.ndarray:
    cv2.rectangle(frame, (detection.x, detection.y), (detection.x + detection.w, detection.y + detection.h), color, 2)
    cv2.putText(frame, detection.label, (detection.x, max(0, detection.y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame


def take_photo(frame: np.ndarray, directory: str, prefix: str = 'photo') -> str:
    os.makedirs(directory, exist_ok=True)
    from time import localtime, strftime, time

    del time
    name = f"{prefix}_{strftime('%Y-%m-%d-%H-%M-%S', localtime())}.jpg"
    path = os.path.join(directory, name)
    cv2.imwrite(path, frame)
    return path


def ensure_display_available(app_name: str) -> None:
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        return
    raise SystemExit(
        f'{app_name} needs a graphical display. '
        'Run it from a desktop session or an X11-forwarded shell.'
    )
