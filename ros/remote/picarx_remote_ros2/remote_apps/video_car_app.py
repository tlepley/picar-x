#!/usr/bin/env python3
"""Remote keyboard + camera app using the ROS image stream."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from time import monotonic

import cv2
from PIL import Image, ImageTk

from ..remote_lib.remote_camera_stream import RemoteCameraStream
from ..remote_lib.remote_picarx import RemotePicarx
from ..remote_lib.vision_utils import ensure_display_available, take_photo

MANUAL = """
Controls:
  Up arrow: Forward
  Left arrow: Turn left
  Down arrow: Backward
  Right arrow: Turn right
  Space: Stop
  +: Increase speed
  -: Decrease speed
  T: Take photo
  H: Print help
  ESC: Quit
"""

HEARTBEAT_SEC = 0.1
MOTION_KEYS = {
    'Up': 'forward',
    'Left': 'turn left',
    'Down': 'backward',
    'Right': 'turn right',
}


def move(car: RemotePicarx, operate: str, speed: int) -> None:
    if operate == 'stop':
        car.stop()
    elif operate == 'forward':
        car.set_dir_servo_angle(0)
        car.forward(speed)
    elif operate == 'backward':
        car.set_dir_servo_angle(0)
        car.backward(speed)
    elif operate == 'turn left':
        car.set_dir_servo_angle(-30)
        car.forward(speed)
    elif operate == 'turn right':
        car.set_dir_servo_angle(30)
        car.forward(speed)


class VideoCarApp:
    def __init__(self) -> None:
        try:
            sys.stdout.reconfigure(line_buffering=True, write_through=True)
        except Exception:
            pass
        ensure_display_available('run_11_video_car')
        self.stream = RemoteCameraStream()
        self.car = RemotePicarx()
        self.speed = 0
        self.status = 'stop'
        self.last_command_at = 0.0
        self.current_frame = None
        self.running = True
        self._pressed_motion_keys: list[str] = []

        if not self.stream.request_start():
            raise SystemExit('Could not start remote camera stream')

        self.car.set_cam_pan_angle(0)
        self.car.set_cam_tilt_angle(0)

        self.root = tk.Tk()
        self.root.title('PI-CAR-X Video Car')
        self.root.configure(bg='#111111')
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self.root.bind_all('<KeyRelease>', self._on_key_release)
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
            text='Arrows drive | Space stop | +/- speed | T photo | H help | Esc quit',
            bg='#111111',
            fg='#f2f2f2',
            font=('TkDefaultFont', 10),
        )
        self.help_label.pack(pady=(0, 8))

        buttons = tk.Frame(self.root, bg='#111111')
        buttons.pack(pady=(0, 12))
        for label, action in [
            ('Stop', self._stop),
            ('Speed +', self._speed_up),
            ('Speed -', self._speed_down),
            ('Photo', self._take_photo),
            ('Help', self._print_manual),
        ]:
            tk.Button(buttons, text=label, command=action, width=10).pack(side=tk.LEFT, padx=4)

        print('Window: PI-CAR-X Video Car')
        print('Keyboard input is captured by the application window. Click it first to give it focus.')
        print(MANUAL, flush=True)

        self.root.after(10, self._tick)
        self.root.after(50, self._focus_video)

    def _install_shortcuts(self) -> None:
        for sequence in (
            '<space>',
            '<KeyPress-plus>',
            '<KeyPress-equal>',
            '<KeyPress-KP_Add>',
            '<KeyPress-minus>',
            '<KeyPress-underscore>',
            '<KeyPress-KP_Subtract>',
            '<KeyPress-t>',
            '<KeyPress-T>',
            '<KeyPress-h>',
            '<KeyPress-H>',
            '<Escape>',
        ):
            self.root.bind_all(sequence, self._on_shortcut)

    def _focus_video(self) -> None:
        try:
            self.root.focus_force()
            self.video_label.focus_set()
        except Exception:
            pass

    def _print_manual(self) -> None:
        print('[HELP] Keyboard shortcuts requested.', flush=True)
        print(MANUAL, flush=True)

    def _speed_up(self) -> None:
        self.speed = min(90, self.speed + 10)
        if self.status != 'stop':
            self._send_command(force=True)

    def _speed_down(self) -> None:
        self.speed = max(0, self.speed - 10)
        if self.speed == 0:
            self.status = 'stop'
        self._send_command(force=True)

    def _stop(self) -> None:
        self.status = 'stop'
        self._send_command(force=True)

    def _take_photo(self) -> None:
        if self.current_frame is None:
            print('[PHOTO] No frame available yet.', flush=True)
            return
        path = take_photo(self.current_frame, os.path.expanduser('~/Pictures/picar-x'))
        print(f'[PHOTO] saved to {path}', flush=True)

    def _on_key_press(self, event) -> None:
        key = event.keysym
        if key == 'Escape':
            self._on_close()
        elif key in MOTION_KEYS:
            self._press_motion_key(key)
        elif key == 'space':
            self._stop()
        elif key in {'plus', 'equal', 'KP_Add'}:
            self._speed_up()
        elif key in {'minus', 'underscore', 'KP_Subtract'}:
            self._speed_down()
        elif key.lower() == 't':
            self._take_photo()
        elif key.lower() == 'h':
            self._print_manual()
        return 'break'

    def _on_shortcut(self, event) -> str:
        key = event.keysym
        if key == 'space':
            self._stop()
        elif key in {'plus', 'equal', 'KP_Add'}:
            self._speed_up()
        elif key in {'minus', 'underscore', 'KP_Subtract'}:
            self._speed_down()
        elif key.lower() == 't':
            self._take_photo()
        elif key.lower() == 'h':
            self._print_manual()
        elif key == 'Escape':
            self._on_close()
        return 'break'

    def _on_key_release(self, event) -> None:
        if event.keysym in MOTION_KEYS:
            self._release_motion_key(event.keysym)
        return 'break'

    def _press_motion_key(self, key: str) -> None:
        if key in self._pressed_motion_keys:
            self._pressed_motion_keys.remove(key)
        self._pressed_motion_keys.append(key)
        self._apply_motion_keys()

    def _release_motion_key(self, key: str) -> None:
        if key in self._pressed_motion_keys:
            self._pressed_motion_keys.remove(key)
        self._apply_motion_keys()

    def _apply_motion_keys(self) -> None:
        if not self._pressed_motion_keys:
            self._stop()
            return
        active_key = self._pressed_motion_keys[-1]
        self._set_motion(MOTION_KEYS[active_key])

    def _set_motion(self, motion: str) -> None:
        if self.speed == 0:
            self.speed = 10
        if motion in {'forward', 'backward'} and self.speed > 60:
            self.speed = 60
        self.status = motion
        self._send_command(force=True)

    def _send_command(self, force: bool = False) -> None:
        now = monotonic()
        if not force and (now - self.last_command_at) < HEARTBEAT_SEC:
            return
        move(self.car, self.status, self.speed)
        self.last_command_at = now

    def _tick(self) -> None:
        if not self.running:
            return
        frame = self.stream.latest_frame()
        if frame is not None:
            self.current_frame = frame
            annotated = frame.copy()
            cv2.putText(annotated, f'status={self.status} speed={self.speed}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, 'Click this window to focus keyboard', (20, annotated.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(image=image)
            self.video_label.configure(image=photo)
            self.video_label.image = photo
        if self.status != 'stop':
            self._send_command()
        self.status_var.set(f'Status: {self.status} | Speed: {self.speed}')
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
    VideoCarApp().run()


if __name__ == '__main__':
    main()
