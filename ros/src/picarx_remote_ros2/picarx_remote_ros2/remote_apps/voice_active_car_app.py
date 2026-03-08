#!/usr/bin/env python3
"""Top-level remote voice-active-car style app for the Jetson side.

This app keeps orchestration on the remote machine while non-driving
embodiment effects stay on the PI-CAR-X through ROS topics.
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from dataclasses import dataclass

from ..remote_lib.remote_embodiment import RemoteEmbodiment
from ..remote_lib.remote_picarx import RemotePicarx


def forward(car: RemotePicarx) -> None:
    car.forward(5)
    time.sleep(1.0)
    car.stop()


def backward(car: RemotePicarx) -> None:
    car.backward(5)
    time.sleep(1.0)
    car.stop()


def wave_hands(car: RemotePicarx) -> None:
    car.reset()
    car.set_cam_tilt_angle(20)
    for _ in range(2):
        car.set_dir_servo_angle(-25)
        time.sleep(0.1)
        car.set_dir_servo_angle(25)
        time.sleep(0.1)
    car.set_dir_servo_angle(0)


def resist(car: RemotePicarx) -> None:
    car.reset()
    car.set_cam_tilt_angle(10)
    for _ in range(3):
        car.set_dir_servo_angle(-15)
        car.set_cam_pan_angle(15)
        time.sleep(0.1)
        car.set_dir_servo_angle(15)
        car.set_cam_pan_angle(-15)
        time.sleep(0.1)
    car.stop()
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)


def act_cute(car: RemotePicarx) -> None:
    car.reset()
    car.set_cam_tilt_angle(-20)
    for _ in range(15):
        car.forward(5)
        time.sleep(0.02)
        car.backward(5)
        time.sleep(0.02)
    car.set_cam_tilt_angle(0)
    car.stop()


def rub_hands(car: RemotePicarx) -> None:
    car.reset()
    for _ in range(5):
        car.set_dir_servo_angle(-6)
        time.sleep(0.5)
        car.set_dir_servo_angle(6)
        time.sleep(0.5)
    car.reset()


def think(car: RemotePicarx) -> None:
    car.reset()
    for i in range(11):
        car.set_cam_pan_angle(i * 3)
        car.set_cam_tilt_angle(-i * 2)
        car.set_dir_servo_angle(i * 2)
        time.sleep(0.05)
    time.sleep(1.0)
    car.set_cam_pan_angle(15)
    car.set_cam_tilt_angle(-10)
    car.set_dir_servo_angle(10)
    time.sleep(0.1)
    car.reset()


def shake_head(car: RemotePicarx) -> None:
    car.stop()
    car.set_cam_pan_angle(0)
    car.set_cam_pan_angle(60)
    time.sleep(0.2)
    car.set_cam_pan_angle(-50)
    time.sleep(0.1)
    car.set_cam_pan_angle(40)
    time.sleep(0.1)
    car.set_cam_pan_angle(-30)
    time.sleep(0.1)
    car.set_cam_pan_angle(20)
    time.sleep(0.1)
    car.set_cam_pan_angle(-10)
    time.sleep(0.1)
    car.set_cam_pan_angle(10)
    time.sleep(0.1)
    car.set_cam_pan_angle(-5)
    time.sleep(0.1)
    car.set_cam_pan_angle(0)


def nod(car: RemotePicarx) -> None:
    car.reset()
    car.set_cam_tilt_angle(0)
    car.set_cam_tilt_angle(5)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-30)
    time.sleep(0.1)
    car.set_cam_tilt_angle(5)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-30)
    time.sleep(0.1)
    car.set_cam_tilt_angle(0)


def depressed(car: RemotePicarx) -> None:
    car.reset()
    car.set_cam_tilt_angle(0)
    car.set_cam_tilt_angle(20)
    time.sleep(0.22)
    car.set_cam_tilt_angle(-22)
    time.sleep(0.1)
    car.set_cam_tilt_angle(10)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-22)
    time.sleep(0.1)
    car.set_cam_tilt_angle(0)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-22)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-10)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-22)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-15)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-22)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-19)
    time.sleep(0.1)
    car.set_cam_tilt_angle(-22)
    time.sleep(0.1)
    time.sleep(1.5)
    car.reset()


def twist_body(car: RemotePicarx) -> None:
    car.reset()
    for _ in range(3):
        car.set_motor_speed(1, 20)
        car.set_motor_speed(2, 20)
        car.set_cam_pan_angle(-20)
        car.set_dir_servo_angle(-10)
        time.sleep(0.1)
        car.set_motor_speed(1, 0)
        car.set_motor_speed(2, 0)
        car.set_cam_pan_angle(0)
        car.set_dir_servo_angle(0)
        time.sleep(0.1)
        car.set_motor_speed(1, -20)
        car.set_motor_speed(2, -20)
        car.set_cam_pan_angle(20)
        car.set_dir_servo_angle(10)
        time.sleep(0.1)
        car.set_motor_speed(1, 0)
        car.set_motor_speed(2, 0)
        car.set_cam_pan_angle(0)
        car.set_dir_servo_angle(0)
        time.sleep(0.1)


def celebrate(car: RemotePicarx) -> None:
    car.reset()
    car.set_cam_tilt_angle(20)
    car.set_dir_servo_angle(30)
    car.set_cam_pan_angle(60)
    time.sleep(0.3)
    car.set_dir_servo_angle(10)
    car.set_cam_pan_angle(30)
    time.sleep(0.1)
    car.set_dir_servo_angle(30)
    car.set_cam_pan_angle(60)
    time.sleep(0.3)
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)
    time.sleep(0.2)
    car.set_dir_servo_angle(-30)
    car.set_cam_pan_angle(-60)
    time.sleep(0.3)
    car.set_dir_servo_angle(-10)
    car.set_cam_pan_angle(-30)
    time.sleep(0.1)
    car.set_dir_servo_angle(-30)
    car.set_cam_pan_angle(-60)
    time.sleep(0.3)
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)
    time.sleep(0.2)


ACTION_FUNCS = {
    'shake head': shake_head,
    'nod': nod,
    'wave hands': wave_hands,
    'resist': resist,
    'act cute': act_cute,
    'rub hands': rub_hands,
    'think': think,
    'twist body': twist_body,
    'celebrate': celebrate,
    'depressed': depressed,
    'forward': forward,
    'backward': backward,
}

SOUND_ACTIONS = {'honking', 'start engine'}
KEYWORD_ACTIONS = (
    ('honk', ['honking']),
    ('horn', ['honking']),
    ('engine', ['start engine']),
    ('start', ['start engine']),
    ('wave', ['wave hands']),
    ('shake', ['shake head']),
    ('nod', ['nod']),
    ('cute', ['act cute']),
    ('celebrate', ['celebrate']),
    ('dance', ['celebrate']),
    ('twist', ['twist body']),
    ('sad', ['depressed']),
    ('depressed', ['depressed']),
    ('resist', ['resist']),
    ('think', ['think']),
    ('back', ['backward']),
    ('reverse', ['backward']),
    ('forward', ['forward']),
)


class RemoteActionFlow:
    def __init__(self, car: RemotePicarx, embodiment: RemoteEmbodiment) -> None:
        self.car = car
        self.embodiment = embodiment
        self._queue: queue.Queue[str] = queue.Queue()
        self._running = False
        self._worker: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._worker = threading.Thread(target=self._run, name='remote_action_flow', daemon=True)
        self._worker.start()

    def stop(self) -> None:
        self._running = False
        if self._worker is not None:
            self._worker.join(timeout=1.0)

    def add_action(self, *actions: str) -> None:
        for action in actions:
            if action in ACTION_FUNCS or action in SOUND_ACTIONS:
                self._queue.put(action)

    def wait_actions_done(self) -> None:
        self._queue.join()

    def _run(self) -> None:
        while self._running:
            try:
                action = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if action in ACTION_FUNCS:
                    ACTION_FUNCS[action](self.car)
                elif action in SOUND_ACTIONS:
                    self.embodiment.play_sound(action)
            finally:
                self._queue.task_done()
                time.sleep(0.2)


@dataclass
class VoiceActiveConfig:
    name: str = 'Rolly'
    too_close_cm: float = 10.0
    wake_word: str = 'hey rolly'
    answer_on_wake: str = 'Hi there'
    welcome: str = "Hi, I'm Rolly. Type to chat with me. Type 'quit' to exit."


class RemoteVoiceActiveCarApp:
    def __init__(self, config: VoiceActiveConfig) -> None:
        self.config = config
        self.car = RemotePicarx(servo_pins=['P0', 'P1', 'P3'])
        self.embodiment = RemoteEmbodiment()
        self.action_flow = RemoteActionFlow(self.car, self.embodiment)

    def run(self) -> None:
        if not sys.stdin.isatty():
            print('This app needs an interactive terminal.')
            print('Run it with: ros2 run picarx_remote_ros2 picarx_voice_active_car_app')
            return

        self.action_flow.start()
        self.embodiment.set_led('off')
        print(self.config.welcome)
        print('This remote split version keeps embodiment on the PI and orchestration on the Jetson.')

        try:
            while True:
                self.embodiment.set_led('blink_once')
                text = input('you> ').strip()
                if not text:
                    continue
                if text.lower() in {'quit', 'exit', 'q'}:
                    break

                response, actions = self._handle_input(text)
                self.embodiment.set_led('on')
                print(f'{self.config.name}> {response}')
                if actions:
                    self.action_flow.add_action(*actions)
                    self.action_flow.wait_actions_done()
                self.embodiment.set_led('off')
        except KeyboardInterrupt:
            pass
        finally:
            self.action_flow.stop()
            self.car.stop()
            self.car.close()
            self.embodiment.set_led('off')

    def _handle_input(self, text: str) -> tuple[str, list[str]]:
        lowered = text.lower()
        actions = self._match_actions(lowered)
        responses = []

        if self.config.wake_word in lowered:
            responses.append(self.config.answer_on_wake)

        distance = self.car.get_distance()
        if 1.0 < distance < self.config.too_close_cm:
            responses.append(f"Obstacle too close at {distance:.1f} centimeters. I'll back up first.")
            actions.insert(0, 'backward')

        if not responses:
            responses.append(self._compose_reply(lowered, actions))

        if not actions:
            actions = ['think']

        return ' '.join(responses), actions

    def _match_actions(self, lowered: str) -> list[str]:
        actions: list[str] = []
        for keyword, mapped in KEYWORD_ACTIONS:
            if keyword in lowered:
                actions.extend(mapped)
        deduped: list[str] = []
        for action in actions:
            if action not in deduped:
                deduped.append(action)
        return deduped

    def _compose_reply(self, lowered: str, actions: list[str]) -> str:
        if 'hello' in lowered or 'hi' in lowered:
            return f'Hello, I am {self.config.name}.'
        if 'status' in lowered:
            distance = self.car.get_distance()
            return f'My last ultrasonic reading is {distance:.1f} centimeters.'
        if actions:
            return f'I will try this: {", ".join(actions)}.'
        return "I did not catch a robot action there, so I'll just think for a moment."


def main() -> None:
    app = RemoteVoiceActiveCarApp(VoiceActiveConfig())
    app.run()


if __name__ == '__main__':
    main()
