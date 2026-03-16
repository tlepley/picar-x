#!/usr/bin/env python3
"""Remote computer-vision demo driven from the ROS camera stream."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from time import localtime, strftime, time

import cv2
import numpy as np
from PIL import Image, ImageTk

from ..remote_lib.remote_camera_stream import RemoteCameraStream
from ..remote_lib.remote_picarx import RemotePicarx
from ..remote_lib.vision_utils import (
    build_cat_detector,
    build_face_detector,
    detect_cat,
    draw_detection,
    ensure_display_available,
)

MANUAL = """
Controls:
  Q: take photo
  1: detect red
  2: detect orange
  3: detect yellow
  4: detect green
  5: detect blue
  6: detect purple
  0: disable color detection
  F: toggle face detection
  C: toggle cat detection
  R: toggle QR detection
  S: print current detection info
  H: print help
  ESC: quit
"""

COLOR_RANGES = {
    'red': [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    'orange': [((10, 120, 70), (22, 255, 255))],
    'yellow': [((22, 120, 70), (35, 255, 255))],
    'green': [((35, 80, 50), (85, 255, 255))],
    'blue': [((90, 80, 50), (130, 255, 255))],
    'purple': [((130, 80, 50), (165, 255, 255))],
}

COLOR_KEY_MAP = {
    '1': 'red',
    '2': 'orange',
    '3': 'yellow',
    '4': 'green',
    '5': 'blue',
    '6': 'purple',
}


class ComputerVisionApp:
    def __init__(self) -> None:
        try:
            sys.stdout.reconfigure(line_buffering=True, write_through=True)
        except Exception:
            pass

        ensure_display_available('run_7_computer_vision')
        self.stream = RemoteCameraStream()
        self.car = RemotePicarx()
        self.flag_face = False
        self.flag_cat = False
        self.flag_color = False
        self.flag_qr = False
        self.color_name = 'close'
        self.last_qr_text: str | None = None
        self.current_frame = None
        self.running = True
        self.last_detection = {
            'color_n': 0,
            'color_x': 0,
            'color_y': 0,
            'color_w': 0,
            'color_h': 0,
            'human_n': 0,
            'human_x': 0,
            'human_y': 0,
            'human_w': 0,
            'human_h': 0,
            'cat_n': 0,
            'cat_x': 0,
            'cat_y': 0,
            'cat_w': 0,
            'cat_h': 0,
            'qr_data': 'None',
        }
        self._face_detector = build_face_detector()
        self._cat_detector = build_cat_detector()
        self._qr_detector = cv2.QRCodeDetector()

        if not self.stream.request_start():
            raise SystemExit('Could not start remote camera stream.')
        self.car.set_cam_pan_angle(0)
        self.car.set_cam_tilt_angle(0)

        self.root = tk.Tk()
        self.root.title('PI-CAR-X Computer Vision')
        self.root.configure(bg='#111111')
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self._install_shortcuts()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self.video_label = tk.Label(self.root, bg='black')
        self.video_label.pack(padx=12, pady=(12, 6))
        self.video_label.bind('<Button-1>', lambda _event: self.video_label.focus_set())

        self.status_var = tk.StringVar()
        self.status_label = tk.Label(
            self.root,
            textvariable=self.status_var,
            bg='#111111',
            fg='#f2f2f2',
            font=('TkDefaultFont', 11),
        )
        self.status_label.pack(pady=(0, 8))

        self.help_label = tk.Label(
            self.root,
            text='Q photo | 1-6 colors | F face | C cat | R qr | S info | H help | Esc quit',
            bg='#111111',
            fg='#f2f2f2',
            font=('TkDefaultFont', 10),
        )
        self.help_label.pack(pady=(0, 8))

        buttons = tk.Frame(self.root, bg='#111111')
        buttons.pack(pady=(0, 12))
        for label, action in [
            ('Photo', self._take_photo),
            ('Face', self._toggle_face),
            ('Cat', self._toggle_cat),
            ('QR', self._toggle_qr),
            ('Info', self.object_show),
            ('Help', self._print_manual),
        ]:
            tk.Button(buttons, text=label, command=action, width=10).pack(side=tk.LEFT, padx=4)

        self._print_manual()
        self.root.after(10, self._tick)
        self.root.after(50, self._focus_video)

    def _install_shortcuts(self) -> None:
        for sequence in (
            '<Escape>',
            '<KeyPress-q>', '<KeyPress-Q>',
            '<KeyPress-f>', '<KeyPress-F>',
            '<KeyPress-c>', '<KeyPress-C>',
            '<KeyPress-r>', '<KeyPress-R>',
            '<KeyPress-s>', '<KeyPress-S>',
            '<KeyPress-h>', '<KeyPress-H>',
            '<KeyPress-0>', '<KeyPress-1>', '<KeyPress-2>', '<KeyPress-3>',
            '<KeyPress-4>', '<KeyPress-5>', '<KeyPress-6>',
        ):
            self.root.bind_all(sequence, self._on_shortcut)

    def _focus_video(self) -> None:
        try:
            self.root.focus_force()
            self.video_label.focus_set()
        except Exception:
            pass

    def _print_manual(self, _event=None) -> str:
        print('Window: PI-CAR-X Computer Vision', flush=True)
        print('Keyboard input is captured by the application window. Click it first to give it focus.', flush=True)
        print(MANUAL, flush=True)
        if self._face_detector is None:
            print('Face detection is unavailable: no OpenCV face cascade was found.', flush=True)
        if self._cat_detector is None:
            print('Cat detection is unavailable: no OpenCV cat cascade was found.', flush=True)
        self._print_active_modes()
        return 'break'

    def _active_modes(self) -> list[str]:
        active_modes = []
        if self.flag_color and self.color_name != 'close':
            active_modes.append(f'color:{self.color_name}')
        if self.flag_face:
            active_modes.append('face')
        if self.flag_cat:
            active_modes.append('cat')
        if self.flag_qr:
            active_modes.append('qr')
        return active_modes

    def _print_active_modes(self) -> None:
        active_modes = self._active_modes()
        print('Active detections:', ', '.join(active_modes) if active_modes else 'none', flush=True)

    def _take_photo(self, _event=None) -> str:
        if self.current_frame is None:
            print('[PHOTO] No frame available yet.', flush=True)
            return 'break'
        name = f"photo_{strftime('%Y-%m-%d-%H-%M-%S', localtime(time()))}.jpg"
        path = os.path.expanduser(f'~/Pictures/{name}')
        cv2.imwrite(path, self.current_frame)
        print(f'[PHOTO] saved to {path}', flush=True)
        return 'break'

    def _set_color_mode(self, color_name: str | None) -> None:
        if color_name is None:
            self.flag_color = False
            self.color_name = 'close'
        else:
            self.flag_color = True
            self.color_name = color_name
        print(f'Color detect : {self.color_name}', flush=True)
        self._print_active_modes()

    def _toggle_face(self, _event=None) -> str:
        self.flag_face = not self.flag_face
        print(f'Face Detect:{self.flag_face}', flush=True)
        self._print_active_modes()
        return 'break'

    def _toggle_cat(self, _event=None) -> str:
        self.flag_cat = not self.flag_cat
        print(f'Cat Detect:{self.flag_cat}', flush=True)
        if self.flag_cat and self._cat_detector is None:
            print('Cat detection requested, but no OpenCV cat cascade is available on this host.', flush=True)
        self._print_active_modes()
        return 'break'

    def _toggle_qr(self, _event=None) -> str:
        self.flag_qr = not self.flag_qr
        print('Waitting for QR code' if self.flag_qr else 'QRcode Detect: close', flush=True)
        self._print_active_modes()
        return 'break'

    def _on_shortcut(self, event) -> str:
        key = event.keysym
        if key == 'Escape':
            self._on_close()
        elif key.lower() == 'q':
            return self._take_photo()
        elif key.lower() == 'f':
            return self._toggle_face()
        elif key.lower() == 'c':
            return self._toggle_cat()
        elif key.lower() == 'r':
            return self._toggle_qr()
        elif key.lower() == 's':
            self.object_show()
        elif key.lower() == 'h':
            return self._print_manual()
        elif key == '0':
            self._set_color_mode(None)
        elif key in COLOR_KEY_MAP:
            self._set_color_mode(COLOR_KEY_MAP[key])
        return 'break'

    def _on_key_press(self, event) -> str:
        if event.keysym == 'Escape':
            self._on_close()
        return 'break'

    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        annotated = frame.copy()
        cv2.putText(
            annotated,
            'Click this window to focus keyboard',
            (20, annotated.shape[0] - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            annotated,
            'Q photo | 1-6 colors | F face | C cat | R qr | S info | H help | Esc quit',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )
        status_text = 'Active detections: ' + (', '.join(self._active_modes()) if self._active_modes() else 'none')
        cv2.putText(
            annotated,
            status_text,
            (20, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        self.last_detection['color_n'] = 0
        self.last_detection['human_n'] = 0
        self.last_detection['cat_n'] = 0
        self.last_detection['qr_data'] = 'None'

        if self.flag_color and self.color_name != 'close':
            self._process_color(frame, annotated)
        if self.flag_face:
            self._process_face(frame, annotated)
        if self.flag_cat:
            self._process_cat(frame, annotated)
        if self.flag_qr:
            self._process_qr(frame, annotated)
        return annotated

    def _process_color(self, frame: np.ndarray, annotated: np.ndarray) -> None:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        for lower, upper in COLOR_RANGES[self.color_name]:
            partial = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = partial if mask is None else cv2.bitwise_or(mask, partial)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 200:
            return
        x, y, w, h = cv2.boundingRect(contour)
        self.last_detection.update({
            'color_n': 1,
            'color_x': x + w // 2,
            'color_y': y + h // 2,
            'color_w': w,
            'color_h': h,
        })
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(annotated, self.color_name, (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def _process_face(self, frame: np.ndarray, annotated: np.ndarray) -> None:
        if self._face_detector is None:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self._face_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
        if len(faces) == 0:
            return
        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        self.last_detection.update({
            'human_n': len(faces),
            'human_x': int(x + w / 2),
            'human_y': int(y + h / 2),
            'human_w': int(w),
            'human_h': int(h),
        })
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 200, 0), 2)
        cv2.putText(annotated, 'face', (x, max(0, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

    def _process_cat(self, frame: np.ndarray, annotated: np.ndarray) -> None:
        detection = detect_cat(frame, self._cat_detector)
        if detection is None:
            return
        self.last_detection.update({
            'cat_n': 1,
            'cat_x': detection.center[0],
            'cat_y': detection.center[1],
            'cat_w': detection.w,
            'cat_h': detection.h,
        })
        draw_detection(annotated, detection, (255, 180, 0))

    def _process_qr(self, frame: np.ndarray, annotated: np.ndarray) -> None:
        text, points, _ = self._qr_detector.detectAndDecode(frame)
        if not text:
            return
        self.last_detection['qr_data'] = text
        if text != self.last_qr_text:
            print(f'QR code:{text}', flush=True)
            self.last_qr_text = text
        if points is not None:
            pts = points.astype(int).reshape(-1, 2)
            for idx in range(len(pts)):
                cv2.line(annotated, tuple(pts[idx]), tuple(pts[(idx + 1) % len(pts)]), (255, 0, 0), 2)
            cv2.putText(annotated, text, tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    def object_show(self) -> None:
        if self.flag_color:
            if self.last_detection['color_n'] == 0:
                print('Color Detect: None', flush=True)
            else:
                print('[Color Detect] Coordinate:',
                      (self.last_detection['color_x'], self.last_detection['color_y']),
                      'Size',
                      (self.last_detection['color_w'], self.last_detection['color_h']),
                      flush=True)
        if self.flag_face:
            if self.last_detection['human_n'] == 0:
                print('Face Detect: None', flush=True)
            else:
                print('[Face Detect] Coordinate:',
                      (self.last_detection['human_x'], self.last_detection['human_y']),
                      'Size',
                      (self.last_detection['human_w'], self.last_detection['human_h']),
                      flush=True)
        if self.flag_cat:
            if self.last_detection['cat_n'] == 0:
                print('Cat Detect: None', flush=True)
            else:
                print('[Cat Detect] Coordinate:',
                      (self.last_detection['cat_x'], self.last_detection['cat_y']),
                      'Size',
                      (self.last_detection['cat_w'], self.last_detection['cat_h']),
                      flush=True)
        if self.flag_qr:
            print('[QR Detect]', self.last_detection['qr_data'], flush=True)

    def _tick(self) -> None:
        if not self.running:
            return
        frame = self.stream.latest_frame()
        if frame is not None:
            self.current_frame = frame
            annotated = self._process_frame(frame)
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(image=image)
            self.video_label.configure(image=photo)
            self.video_label.image = photo
        active = ', '.join(self._active_modes()) if self._active_modes() else 'none'
        self.status_var.set(f'Active detections: {active}')
        self.root.after(15, self._tick)

    def _on_close(self) -> None:
        if not self.running:
            return
        self.running = False
        self.car.stop()
        self.stream.request_stop()
        self.stream.close()
        self.root.destroy()

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            if self.running:
                self._on_close()


def main() -> None:
    ComputerVisionApp().run()


if __name__ == '__main__':
    main()
