#!/usr/bin/env python3
"""Simple tkinter GUI for remote PI-CAR-X calibration."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox

import rclpy

from .calibration_cli import CalibrationClient


class CalibrationGui:
    def __init__(self, root: tk.Tk, node: CalibrationClient) -> None:
        self.root = root
        self.node = node
        self.root.title('PI-CAR-X Remote Calibration')

        self.dir_var = tk.StringVar(value='0.0')
        self.pan_var = tk.StringVar(value='0.0')
        self.tilt_var = tk.StringVar(value='0.0')
        self.left_var = tk.StringVar(value='1')
        self.right_var = tk.StringVar(value='1')
        self.status_var = tk.StringVar(value='Ready')

        self._build_ui()
        self._pump_ros()
        self.refresh_state()

    def _build_ui(self) -> None:
        pad = {'padx': 6, 'pady': 4}

        servo = tk.LabelFrame(self.root, text='Servo Offsets (dir, pan, tilt)')
        servo.pack(fill='x', padx=8, pady=6)
        tk.Label(servo, text='Dir').grid(row=0, column=0, **pad)
        tk.Entry(servo, textvariable=self.dir_var, width=8).grid(row=0, column=1, **pad)
        tk.Label(servo, text='Pan').grid(row=0, column=2, **pad)
        tk.Entry(servo, textvariable=self.pan_var, width=8).grid(row=0, column=3, **pad)
        tk.Label(servo, text='Tilt').grid(row=0, column=4, **pad)
        tk.Entry(servo, textvariable=self.tilt_var, width=8).grid(row=0, column=5, **pad)
        tk.Button(servo, text='Apply Servo', command=self.apply_servo).grid(row=0, column=6, **pad)

        motor = tk.LabelFrame(self.root, text='Motor Directions (left, right)')
        motor.pack(fill='x', padx=8, pady=6)
        tk.Label(motor, text='Left').grid(row=0, column=0, **pad)
        tk.OptionMenu(motor, self.left_var, '-1', '1').grid(row=0, column=1, **pad)
        tk.Label(motor, text='Right').grid(row=0, column=2, **pad)
        tk.OptionMenu(motor, self.right_var, '-1', '1').grid(row=0, column=3, **pad)
        tk.Button(motor, text='Apply Motor', command=self.apply_motor).grid(row=0, column=4, **pad)

        actions = tk.Frame(self.root)
        actions.pack(fill='x', padx=8, pady=8)
        tk.Button(actions, text='Save', width=10, command=lambda: self.call_service('save')).pack(side='left', padx=4)
        tk.Button(actions, text='Load', width=10, command=lambda: self.call_service('load')).pack(side='left', padx=4)
        tk.Button(actions, text='Reset', width=10, command=lambda: self.call_service('reset')).pack(side='left', padx=4)
        tk.Button(actions, text='Refresh', width=10, command=self.refresh_state).pack(side='left', padx=4)

        tk.Label(self.root, textvariable=self.status_var, anchor='w').pack(fill='x', padx=10, pady=6)

    def _pump_ros(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(50, self._pump_ros)

    def _parse_float(self, value: str) -> float:
        try:
            return float(value)
        except ValueError as exc:
            raise ValueError(f'Invalid number: {value}') from exc

    def apply_servo(self) -> None:
        try:
            d = self._parse_float(self.dir_var.get())
            p = self._parse_float(self.pan_var.get())
            t = self._parse_float(self.tilt_var.get())
            self.node.set_servo_offsets(d, p, t)
            self.status_var.set(f'Applied servo offsets: {d}, {p}, {t}')
            self.root.after(200, self.refresh_state)
        except Exception as exc:
            messagebox.showerror('Servo error', str(exc))

    def apply_motor(self) -> None:
        try:
            left = int(self.left_var.get())
            right = int(self.right_var.get())
            self.node.set_motor_directions(left, right)
            self.status_var.set(f'Applied motor directions: {left}, {right}')
            self.root.after(200, self.refresh_state)
        except Exception as exc:
            messagebox.showerror('Motor error', str(exc))

    def call_service(self, name: str) -> None:
        ok = self.node.call_trigger(name)
        if ok:
            self.status_var.set(f'Service {name} ok')
            self.refresh_state()
        else:
            self.status_var.set(f'Service {name} failed')
            messagebox.showerror('Service error', f'/picarx/calibration/{name} failed')

    def refresh_state(self) -> None:
        message = self.node.get_state_message()
        if message is None:
            self.status_var.set('Failed to read calibration state')
            return
        fields = {}
        for token in message.split():
            if '=' in token:
                key, value = token.split('=', 1)
                fields[key] = value
        self.dir_var.set(fields.get('dir_offset', self.dir_var.get()))
        self.pan_var.set(fields.get('pan_offset', self.pan_var.get()))
        self.tilt_var.set(fields.get('tilt_offset', self.tilt_var.get()))
        self.left_var.set(fields.get('left_dir', self.left_var.get()))
        self.right_var.set(fields.get('right_dir', self.right_var.get()))
        self.status_var.set('State refreshed')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrationClient()
    root = tk.Tk()
    app = CalibrationGui(root, node)
    del app

    try:
        root.mainloop()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
