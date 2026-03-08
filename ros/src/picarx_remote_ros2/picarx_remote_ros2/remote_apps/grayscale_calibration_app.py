#!/usr/bin/env python3
"""Interactive remote grayscale calibration app for PI-CAR-X."""

from __future__ import annotations

import sys
import threading
import time
import termios
import tty

from .calibration_cli import CalibrationClient
from ..remote_lib.remote_picarx import RemotePicarx

MANUAL = """\
        ┌────────────────────────────────────┐
        │ Picar-X Grayscale Module Reference │
        │       Calibration Helper           │
        └────────────────────────────────────┘

 press [Q] to start line reference calibration,
 press [E] to start cliff reference calibration

 [SPACE]: confirm calibration           [Ctrl+C]: quit
"""


def read_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class GrayscaleCalibrationApp:
    def __init__(self) -> None:
        self.car = RemotePicarx()
        self.cli = CalibrationClient()
        self.current_grayscale_value = [0.0, 0.0, 0.0]
        self.line_reference = list(self.car.line_reference)
        self.cliff_reference = list(self.car.cliff_reference)
        self.current_mode: str | None = None
        self.thresholds = [[4096.0, 0.0], [4096.0, 0.0], [4096.0, 0.0]]
        self.run_flag = False
        self.cali_status = 'none'

    def update_info(self, isback: bool = True) -> None:
        if isback:
            print("\033[6A", end="\r")
        if self.current_mode is None:
            print("\033[K\033[32m ---------- \033[m")
        elif self.current_mode == 'line_cali':
            print("\033[K\033[33mLine reference auto calibrating ...\033[m")
        elif self.current_mode == 'line_cali_done':
            print("\033[K\033[32mLine reference auto calibration done.\033[m")
        elif self.current_mode == 'cliff_cali':
            print("\033[K\033[33mCliff reference auto calibrating ...\033[m")
        elif self.current_mode == 'cliff_cali_done':
            print("\033[K\033[32mCliff reference auto calibration done.\033[m")
        elif self.current_mode == 'saved':
            print("\033[K\033[32mThe reference values have been saved.\033[m")
        else:
            print("\033[K")
        print("\033[K")
        print(f"\033[Kcurrent value: {self.current_grayscale_value}")
        print(f"\033[Kthresholds: {self.thresholds}")
        print(f"\033[Kline reference: {self.line_reference}")
        print(f"\033[Kcliff reference: {self.cliff_reference}")

    def read_data_loop(self) -> None:
        while self.run_flag:
            self.current_grayscale_value = self.car.get_grayscale_data()
            if self.cali_status == 'work':
                for i in range(3):
                    self.thresholds[i][0] = min(self.thresholds[i][0], self.current_grayscale_value[i])
                    self.thresholds[i][1] = max(self.thresholds[i][1], self.current_grayscale_value[i])
                    self.line_reference[i] = int((self.thresholds[i][0] + self.thresholds[i][1]) / 2)
            elif self.cali_status == 'done':
                if all(self.cliff_reference[i] < self.line_reference[i] for i in range(3)):
                    for i in range(3):
                        self.cliff_reference[i] = int((self.cliff_reference[i] + self.line_reference[i]) / 2)
                self.cali_status = 'none'
            time.sleep(0.2)

    def start_line_calibrate(self) -> None:
        def work() -> None:
            self.current_mode = 'line_cali'
            self.cali_status = 'work'
            self.thresholds = [[4096.0, 0.0], [4096.0, 0.0], [4096.0, 0.0]]
            angle = 35
            delay = 0.8
            self.car.set_dir_servo_angle(-angle)
            self.car.forward(10)
            time.sleep(delay)
            self.car.backward(10)
            time.sleep(delay)
            self.car.set_dir_servo_angle(0)
            self.car.stop()
            time.sleep(0.2)
            self.car.set_dir_servo_angle(angle)
            self.car.forward(10)
            time.sleep(delay)
            self.car.backward(10)
            time.sleep(delay)
            self.car.set_dir_servo_angle(0)
            self.car.stop()
            time.sleep(0.2)
            self.current_mode = 'line_cali_done'
            self.cali_status = 'done'

        threading.Thread(target=work, daemon=True).start()

    def start_cliff_calibrate(self) -> None:
        def work() -> None:
            self.current_mode = 'cliff_cali'
            acc = [0.0, 0.0, 0.0]
            for _ in range(10):
                for i in range(3):
                    acc[i] += self.current_grayscale_value[i]
                time.sleep(0.2)
            vals = [v / 10.0 for v in acc]
            if all(vals[i] < self.thresholds[i][0] for i in range(3)):
                vals = [int((vals[i] + self.thresholds[i][0]) / 2) for i in range(3)]
            self.cliff_reference = [int(v) for v in vals]
            self.current_mode = 'cliff_cali_done'

        threading.Thread(target=work, daemon=True).start()

    def run(self) -> None:
        if not sys.stdin.isatty():
            print("This app needs an interactive terminal.")
            print("Run it with: ros2 run picarx_remote_ros2 run_1_cali_grayscale_app")
            raise SystemExit(1)

        self.run_flag = True
        threading.Thread(target=self.read_data_loop, daemon=True).start()
        print(MANUAL)
        self.update_info(False)
        try:
            while self.run_flag:
                key = read_key().lower()
                if key == 'q':
                    self.start_line_calibrate()
                elif key == 'e':
                    self.start_cliff_calibrate()
                elif key == ' ':
                    print('\nConfirm save ?(y/n)')
                    while True:
                        confirm = read_key().lower()
                        if confirm == 'y':
                            self.car.set_line_reference(self.line_reference)
                            self.car.set_cliff_reference(self.cliff_reference)
                            self.cli.call_trigger('save')
                            self.current_mode = 'saved'
                            break
                        if confirm == 'n':
                            self.current_mode = None
                            break
                elif key == '\x03':
                    print("\nquit")
                    break
                self.update_info()
        finally:
            self.run_flag = False
            self.car.stop()
            self.cli.destroy_node()
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()


def main() -> None:
    import rclpy

    rclpy.init(args=None)
    app = GrayscaleCalibrationApp()
    app.run()


if __name__ == "__main__":
    main()
