#!/usr/bin/env python3
"""Remote video recording app using the ROS image stream."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from time import strftime
from typing import Optional

import cv2
from PIL import Image, ImageTk

from ..remote_lib.remote_camera_stream import RemoteCameraStream
from ..remote_lib.remote_picarx import RemotePicarx
from ..remote_lib.vision_utils import ensure_display_available

MANUAL = """
Controls:
  Q: start / pause / continue recording
  E: stop recording
  H: print help
  ESC: quit
"""


class RecordVideoApp:
    def __init__(self) -> None:
        try:
            sys.stdout.reconfigure(line_buffering=True, write_through=True)
        except Exception:
            pass

        ensure_display_available('run_9_record_video')
        self.stream = RemoteCameraStream()
        self.car = RemotePicarx()
        self.writer: Optional[cv2.VideoWriter] = None
        self.record_state = 'stop'
        self.current_frame = None
        self.running = True
        self.current_output_path: Optional[str] = None

        os.makedirs(os.path.expanduser('~/Videos'), exist_ok=True)
        if not self.stream.request_start():
            raise SystemExit('Could not start remote camera stream')
        self.car.set_cam_pan_angle(0)
        self.car.set_cam_tilt_angle(0)

        self.root = tk.Tk()
        self.root.title('PI-CAR-X Record Video')
        self.root.configure(bg='#111111')
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self.root.bind_all('<Escape>', lambda _event: self._on_close())
        self.root.bind_all('<KeyPress-q>', self._toggle_recording)
        self.root.bind_all('<KeyPress-Q>', self._toggle_recording)
        self.root.bind_all('<KeyPress-e>', self._stop_recording)
        self.root.bind_all('<KeyPress-E>', self._stop_recording)
        self.root.bind_all('<KeyPress-h>', self._print_manual)
        self.root.bind_all('<KeyPress-H>', self._print_manual)
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
            text='Q start/pause | E stop | H help | Esc quit',
            bg='#111111',
            fg='#f2f2f2',
            font=('TkDefaultFont', 10),
        )
        self.help_label.pack(pady=(0, 8))

        buttons = tk.Frame(self.root, bg='#111111')
        buttons.pack(pady=(0, 12))
        for label, action in [
            ('Start/Pause', self._toggle_recording),
            ('Stop', self._stop_recording),
            ('Help', self._print_manual),
            ('Quit', self._on_close),
        ]:
            tk.Button(buttons, text=label, command=action, width=12).pack(side=tk.LEFT, padx=4)

        print('Window: PI-CAR-X Record Video')
        print('Keyboard input is captured by the application window. Click it first to give it focus.')
        print(MANUAL, flush=True)

        self.root.after(10, self._tick)
        self.root.after(50, self._focus_video)

    def _focus_video(self) -> None:
        try:
            self.root.focus_force()
            self.video_label.focus_set()
        except Exception:
            pass

    def _print_manual(self, _event=None) -> str:
        print('[HELP] Keyboard shortcuts requested.', flush=True)
        print(MANUAL, flush=True)
        return 'break'

    def _toggle_recording(self, _event=None) -> str:
        if self.current_frame is None:
            print('[REC] No frame available yet.', flush=True)
            return 'break'
        if self.record_state == 'stop':
            self.current_output_path = os.path.expanduser(f"~/Videos/{strftime('%Y-%m-%d-%H.%M.%S')}.avi")
            self.writer = cv2.VideoWriter(
                self.current_output_path,
                cv2.VideoWriter_fourcc(*'XVID'),
                20.0,
                (self.current_frame.shape[1], self.current_frame.shape[0]),
            )
            self.record_state = 'start'
            print(f'[REC] started: {self.current_output_path}', flush=True)
        elif self.record_state == 'start':
            self.record_state = 'pause'
            print('[REC] paused', flush=True)
        elif self.record_state == 'pause':
            self.record_state = 'start'
            print('[REC] continued', flush=True)
        return 'break'

    def _stop_recording(self, _event=None) -> str:
        if self.record_state != 'stop':
            self.record_state = 'stop'
            if self.writer is not None:
                self.writer.release()
                self.writer = None
            print('[REC] stopped', flush=True)
        return 'break'

    def _tick(self) -> None:
        if not self.running:
            return
        frame = self.stream.latest_frame()
        if frame is not None:
            self.current_frame = frame
            if self.record_state == 'start' and self.writer is not None:
                self.writer.write(frame)
            annotated = frame.copy()
            cv2.putText(annotated, f'recording={self.record_state}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, 'Click this window to focus keyboard', (20, annotated.shape[0] - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            photo = ImageTk.PhotoImage(image=image)
            self.video_label.configure(image=photo)
            self.video_label.image = photo
        self.status_var.set(f'Record state: {self.record_state}')
        self.root.after(15, self._tick)

    def _on_key_press(self, event) -> str:
        if event.keysym == 'Escape':
            self._on_close()
        return 'break'

    def _on_close(self) -> None:
        if not self.running:
            return
        self.running = False
        self._stop_recording()
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
    RecordVideoApp().run()


if __name__ == '__main__':
    main()
