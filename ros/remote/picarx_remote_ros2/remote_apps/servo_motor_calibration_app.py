#!/usr/bin/env python3
"""Interactive remote servo and motor calibration app for PI-CAR-X."""

from __future__ import annotations

import sys
import termios
import tty
from time import sleep

from .calibration_cli import CalibrationClient
from ..remote_lib.remote_picarx import RemotePicarx

MANUAL = """
--------------- Picar-X Calibration Helper -----------------

    [1]: direction servo            [W/D]: increase servo angle
    [2]: camera pan servo           [S/A]: decrease servo angle
    [3]: camera tilt servo          [R]: servos test

    [4]: left motor                 [Q]: change motor direction
    [5]: right motor                [E]: motors run/stop

    [SPACE]: confirm calibration                [Ctrl+C]: quit
"""


def read_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def show_info(servo_num: int, motor_num: int, servos_offset: list[float], motors_offset: list[int]) -> None:
    print("\033[H\033[J", end="")
    print(MANUAL)
    print(f"[ {['direction servo', 'camera pan servo', 'camera tilt servo'][servo_num]} ] "
          f"[ {['left motor', 'right motor'][motor_num]} ]")
    print(f"offset: {servos_offset}, {motors_offset}")


def servos_test(car: RemotePicarx) -> None:
    for value in (-30, 30, -10, 10, 0):
        car.set_dir_servo_angle(value)
        sleep(0.5)
    for value in (-30, 30, 0):
        car.set_cam_pan_angle(value)
        sleep(0.5)
    for value in (-30, 30, 0):
        car.set_cam_tilt_angle(value)
        sleep(0.5)


def apply_servo_position(car: RemotePicarx, servo_num: int) -> None:
    if servo_num == 0:
        car.set_dir_servo_angle(0)
    elif servo_num == 1:
        car.set_cam_pan_angle(0)
    else:
        car.set_cam_tilt_angle(0)
    sleep(0.2)


def main(args=None) -> None:
    del args
    if not sys.stdin.isatty():
        print("This app needs an interactive terminal.")
        print("Run it with: ros2 run picarx_remote_ros2 run_1_cali_servo_motor_app")
        raise SystemExit(1)

    import rclpy

    rclpy.init(args=None)
    cli = CalibrationClient()
    car = RemotePicarx()
    px_power = 30
    servo_num = 0
    motor_num = 0
    servos_offset = [0.0, 0.0, 0.0]
    motors_offset = [1, 1]
    motor_run = False
    step = 0.4

    try:
        message = cli.get_state_message()
        if message:
            fields = {}
            for token in message.split():
                if '=' in token:
                    key, value = token.split('=', 1)
                    fields[key] = value
            servos_offset = [
                float(fields.get('dir_offset', '0')),
                float(fields.get('pan_offset', '0')),
                float(fields.get('tilt_offset', '0')),
            ]
            motors_offset = [
                int(fields.get('left_dir', '1')),
                int(fields.get('right_dir', '1')),
            ]

        car.reset()
        show_info(servo_num, motor_num, servos_offset, motors_offset)

        while True:
            key = read_key().lower()
            if key in '123':
                servo_num = int(key) - 1
            elif key in '45':
                motor_num = int(key) - 4
            elif key == 'r':
                servos_test(car)
            elif key in ('w', 'd'):
                servos_offset[servo_num] = min(20.0, round(servos_offset[servo_num] + step, 2))
                cli.set_servo_offsets(*servos_offset)
                apply_servo_position(car, servo_num)
            elif key in ('s', 'a'):
                servos_offset[servo_num] = max(-20.0, round(servos_offset[servo_num] - step, 2))
                cli.set_servo_offsets(*servos_offset)
                apply_servo_position(car, servo_num)
            elif key == 'q':
                motors_offset[motor_num] = -1 * motors_offset[motor_num]
                cli.set_motor_directions(*motors_offset)
                motor_run = True
                car.forward(px_power)
            elif key == 'e':
                motor_run = not motor_run
                if motor_run:
                    car.forward(px_power)
                else:
                    car.stop()
            elif key == ' ':
                print("\nConfirm save ?(y/n)")
                while True:
                    confirm = read_key().lower()
                    if confirm == 'y':
                        cli.call_trigger('save')
                        break
                    if confirm == 'n':
                        break
            elif key == '\x03':
                print("\nquit")
                break

            show_info(servo_num, motor_num, servos_offset, motors_offset)
            sleep(0.01)
    finally:
        car.reset()
        car.stop()
        cli.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
