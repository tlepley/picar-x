#!/usr/bin/env python3
"""Interactive remote keyboard control app for PI-CAR-X."""

from __future__ import annotations

import sys
import termios
import tty
from time import sleep

from ..remote_lib.remote_picarx import RemotePicarx

MANUAL = """
Press keys on keyboard to control PiCar-X!
    w: Forward
    a: Turn left
    s: Backward
    d: Turn right
    i: Head up
    k: Head down
    j: Turn head left
    l: Turn head right
    ctrl+c: Press twice to exit the program
"""


def show_info() -> None:
    print("\033[H\033[J", end="")
    print(MANUAL)


def read_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main() -> None:
    if not sys.stdin.isatty():
        print("This app needs an interactive terminal.")
        print("Run it with: ros2 run picarx_remote_ros2 run_3_keyboard_control_app")
        raise SystemExit(1)

    pan_angle = 0
    tilt_angle = 0
    car = RemotePicarx()

    try:
        show_info()
        while True:
            key = read_key().lower()
            if key in "wsadikjl":
                if key == "w":
                    car.set_dir_servo_angle(0)
                    car.forward(80)
                    sleep(0.5)
                    car.stop()
                elif key == "s":
                    car.set_dir_servo_angle(0)
                    car.backward(80)
                    sleep(0.5)
                    car.stop()
                elif key == "a":
                    car.set_dir_servo_angle(-30)
                    car.forward(80)
                    sleep(0.5)
                    car.stop()
                elif key == "d":
                    car.set_dir_servo_angle(30)
                    car.forward(80)
                    sleep(0.5)
                    car.stop()
                elif key == "i":
                    tilt_angle = min(30, tilt_angle + 5)
                elif key == "k":
                    tilt_angle = max(-30, tilt_angle - 5)
                elif key == "l":
                    pan_angle = min(30, pan_angle + 5)
                elif key == "j":
                    pan_angle = max(-30, pan_angle - 5)

                car.set_cam_tilt_angle(tilt_angle)
                car.set_cam_pan_angle(pan_angle)
                show_info()
            elif key == "\x03":
                print("\nQuit")
                break
    finally:
        car.set_cam_tilt_angle(0)
        car.set_cam_pan_angle(0)
        car.set_dir_servo_angle(0)
        car.stop()
        sleep(0.2)


if __name__ == "__main__":
    main()
