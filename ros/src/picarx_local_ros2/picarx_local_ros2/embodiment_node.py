#!/usr/bin/env python3
"""ROS node for non-driving robot embodiment features on the PI-CAR-X."""

from __future__ import annotations

import enum
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from robot_hat.led import LED


class PicarxEmbodimentNode(Node):
    """Handles LED and sound effects that must stay local to the PI-CAR-X."""

    def __init__(self) -> None:
        super().__init__('picarx_embodiment_node')

        self.declare_parameter('enable_audio', False)
        self._enable_audio = bool(self.get_parameter('enable_audio').value)
        self._led = LED()
        self._music: Optional[object] = None
        share_dir = Path(get_package_share_directory('picarx_local_ros2'))
        self._sounds_dir = share_dir / 'sounds'
        self._sound_map = {
            'honking': self._sounds_dir / 'car-double-horn.wav',
            'start engine': self._sounds_dir / 'car-start-engine.wav',
        }

        self.create_subscription(String, '/picarx/embodiment/led', self._on_led_command, 10)
        self.create_subscription(String, '/picarx/embodiment/sound', self._on_sound_command, 10)
        self.create_subscription(String, '/picarx/embodiment/speech', self._on_speech_command, 10)

        self.get_logger().info(
            f'PI-CAR-X embodiment node started (LED + sounds, enable_audio={self._enable_audio})'
        )

    def _ensure_music(self):
        if self._music is None:
            from robot_hat.music import Music

            self._music = Music()
        return self._music

    def _on_led_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        self.get_logger().info(f"LED command received: {command}")

        if command == 'on':
            self._led.on()
            return
        if command == 'off':
            self._led.off()
            return
        if command == 'blink_once':
            self._led.blink(times=2, delay=0.1, pause=0.8)
            self._led.off()
            return
        if command == 'blink_fast':
            self._led.blink(times=6, delay=0.1, pause=0.0)
            return

        self.get_logger().warning(f'Unknown LED command: {command}')

    def _on_sound_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        self.get_logger().info(f"Sound command received: {command}")
        if not self._enable_audio:
            self.get_logger().warning('Ignoring sound command because audio is disabled')
            return
        sound_path = self._sound_map.get(command)
        if sound_path is None:
            self.get_logger().warning(f'Unknown sound command: {command}')
            return
        if not sound_path.exists():
            self.get_logger().error(f'Sound asset not found: {sound_path}')
            return

        volume = 100 if command == 'honking' else 50
        try:
            self._ensure_music().sound_play_threading(str(sound_path), volume)
        except Exception as exc:
            self.get_logger().error(f'Failed to play sound {sound_path}: {exc}')

    def _on_speech_command(self, msg: String) -> None:
        payload = msg.data
        self.get_logger().info('Speech command received')
        if not self._enable_audio:
            self.get_logger().warning('Ignoring speech command because audio is disabled')
            return

        engine_name = 'espeak'
        text = payload
        if '|' in payload:
            engine_name, text = payload.split('|', 1)
            engine_name = engine_name.strip().lower() or 'espeak'
        text = text.strip()
        if not text:
            self.get_logger().warning('Ignoring empty speech command')
            return

        try:
            if not hasattr(enum, 'StrEnum'):
                class StrEnum(str, enum.Enum):
                    pass

                enum.StrEnum = StrEnum

            if engine_name == 'pico2wave':
                from robot_hat.tts import Pico2Wave

                speaker = Pico2Wave()
            else:
                from robot_hat.tts import Espeak

                speaker = Espeak()
            speaker.say(text)
        except Exception as exc:
            self.get_logger().error(f'Failed to speak with {engine_name}: {exc}')

    def destroy_node(self) -> bool:
        try:
            self._led.off()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PicarxEmbodimentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
