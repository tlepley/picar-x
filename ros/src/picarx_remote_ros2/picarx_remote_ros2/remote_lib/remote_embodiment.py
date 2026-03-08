#!/usr/bin/env python3
"""Remote client for PI-CAR-X embodiment topics."""

from __future__ import annotations

from std_msgs.msg import String

from .remote_runtime import get_runtime


class RemoteEmbodiment:
    """Remote publisher for LED and sound effects handled on the PI."""

    def __init__(self) -> None:
        self._ctx = get_runtime()

    def set_led(self, command: str) -> None:
        msg = String()
        msg.data = str(command)
        self._ctx.led_pub.publish(msg)

    def play_sound(self, name: str) -> None:
        msg = String()
        msg.data = str(name)
        self._ctx.sound_pub.publish(msg)

    def say_text(self, text: str, *, engine: str = 'espeak') -> None:
        msg = String()
        msg.data = f'{engine}|{text}'
        self._ctx.speech_pub.publish(msg)
