#!/usr/bin/env python3
"""Remote treasure-hunt app using color detection from the ROS image stream."""

from __future__ import annotations

import random
import sys
import tkinter as tk
from time import monotonic

import cv2
from PIL import Image, ImageTk

from ..remote_lib.remote_camera_stream import RemoteCameraStream
from ..remote_lib.remote_embodiment import RemoteEmbodiment
from ..remote_lib.remote_picarx import RemotePicarx
from ..remote_lib.vision_utils import detect_color, draw_detection, ensure_display_available

COLORS = ["red", "orange", "yellow", "green", "blue", "purple"]
DETECTION_WIDTH_THRESHOLD = 100
DRIVE_SPEED = 80
TURN_ANGLE = 30
HEARTBEAT_SEC = 0.1
MOTION_KEYS = {
    'Up': 'forward',
    'Left': 'turn left',
    'Down': 'backward',
    'Right': 'turn right',
}

MANUAL = """
Controls:
  Up arrow: Forward
  Left arrow: Turn left
  Down arrow: Backward
  Right arrow: Turn right
  Space: Repeat the current target color
  N: Choose a new target color
  H: Print help
  ESC: Quit
"""


def say(embodiment: RemoteEmbodiment, line: str) -> None:
    print(f'[SAY] {line}', flush=True)
    embodiment.say_text(line, engine='pico2wave')


def move(car: RemotePicarx, operate: str) -> None:
    if operate == 'stop':
        car.stop()
    elif operate == 'forward':
        car.set_dir_servo_angle(0)
        car.forward(DRIVE_SPEED)
    elif operate == 'backward':
        car.set_dir_servo_angle(0)
        car.backward(DRIVE_SPEED)
    elif operate == 'turn left':
        car.set_dir_servo_angle(-TURN_ANGLE)
        car.forward(DRIVE_SPEED)
    elif operate == 'turn right':
        car.set_dir_servo_angle(TURN_ANGLE)
        car.forward(DRIVE_SPEED)


class TreasureHuntApp:
    def __init__(self) -> None:
        try:
            sys.stdout.reconfigure(line_buffering=True, write_through=True)
        except Exception:
            pass

        ensure_display_available('run_20_treasure_hunt')
        self.stream = RemoteCameraStream()
        self.car = RemotePicarx()
        self.embodiment = RemoteEmbodiment()
        self.current_color = random.choice(COLORS)
        self.current_frame = None
        self.running = True
        self.status = 'stop'
        self.last_command_at = 0.0
        self._pressed_motion_keys: list[str] = []

        if not self.stream.request_start():
            raise SystemExit('Could not start remote camera stream')

        self.root = tk.Tk()
        self.root.title('PI-CAR-X Treasure Hunt')
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
            text='Arrows move | Space repeat color | N new color | H help | Esc quit',
            bg='#111111',
            fg='#f2f2f2',
            font=('TkDefaultFont', 10),
        )
        self.help_label.pack(pady=(0, 8))

        buttons = tk.Frame(self.root, bg='#111111')
        buttons.pack(pady=(0, 12))
        for label, action in [
            ('Repeat', self._repeat_target),
            ('New Color', self._new_target_color),
            ('Stop', self._stop),
            ('Help', self._print_manual),
        ]:
            tk.Button(buttons, text=label, command=action, width=10).pack(side=tk.LEFT, padx=4)

        print('Window: PI-CAR-X Treasure Hunt')
        print('Keyboard input is captured by the application window. Click it first to give it focus.')
        print(MANUAL, flush=True)

        say(self.embodiment, 'Game start!')
        self._announce_target()

        self.root.after(10, self._tick)
        self.root.after(50, self._focus_video)

    def _install_shortcuts(self) -> None:
        for sequence in (
            '<space>',
            '<KeyPress-h>', '<KeyPress-H>',
            '<KeyPress-n>', '<KeyPress-N>',
            '<Escape>',
        ):
            self.root.bind_all(sequence, self._on_shortcut)

    def _focus_video(self) -> None:
        try:
            self.root.focus_force()
            self.video_label.focus_set()
        except Exception:
            pass

    def _announce_target(self) -> None:
        say(self.embodiment, f'Look for {self.current_color}!')

    def _repeat_target(self, _event=None) -> str:
        self._announce_target()
        return 'break'

    def _new_target_color(self, _event=None) -> str:
        self.current_color = random.choice(COLORS)
        print(f'[TARGET] new color: {self.current_color}', flush=True)
        self._announce_target()
        return 'break'

    def _print_manual(self, _event=None) -> str:
        print('[HELP] Keyboard shortcuts requested.', flush=True)
        print(MANUAL, flush=True)
        return 'break'

    def _on_shortcut(self, event) -> str:
        key = event.keysym
        if key == 'space':
            return self._repeat_target()
        if key.lower() == 'n':
            return self._new_target_color()
        if key.lower() == 'h':
            return self._print_manual()
        if key == 'Escape':
            self._on_close()
        return 'break'

    def _on_key_press(self, event) -> str:
        key = event.keysym
        if key == 'Escape':
            self._on_close()
        elif key in MOTION_KEYS:
            self._press_motion_key(key)
        return 'break'

    def _on_key_release(self, event) -> str:
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
        self.status = MOTION_KEYS[active_key]
        self._send_command(force=True)

    def _stop(self) -> None:
        self.status = 'stop'
        self._send_command(force=True)

    def _send_command(self, force: bool = False) -> None:
        now = monotonic()
        if not force and (now - self.last_command_at) < HEARTBEAT_SEC:
            return
        move(self.car, self.status)
        self.last_command_at = now

    def _tick(self) -> None:
        if not self.running:
            return
        frame = self.stream.latest_frame()
        if frame is not None:
            self.current_frame = frame
            annotated = frame.copy()
            detection = detect_color(frame, self.current_color)
            if detection is not None:
                draw_detection(annotated, detection, (0, 255, 255))
                if detection.w > DETECTION_WIDTH_THRESHOLD:
                    say(self.embodiment, 'Well done!')
                    self.current_color = random.choice(COLORS)
                    self._announce_target()
            cv2.putText(annotated, f'target={self.current_color} status={self.status}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, 'Click this window to focus keyboard', (20, annotated.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(image=image)
            self.video_label.configure(image=photo)
            self.video_label.image = photo
        if self.status != 'stop':
            self._send_command()
        self.status_var.set(f'Target: {self.current_color} | Status: {self.status}')
        self.root.after(15, self._tick)

    def _on_close(self) -> None:
        if not self.running:
            return
        self.running = False
        self.car.stop()
        say(self.embodiment, 'Goodbye!')
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
    TreasureHuntApp().run()


if __name__ == '__main__':
    main()
