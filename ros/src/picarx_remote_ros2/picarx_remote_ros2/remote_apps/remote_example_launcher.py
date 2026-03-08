#!/usr/bin/env python3
"""Run PI-CAR-X examples with RemotePicarx patched in."""

from __future__ import annotations

import argparse
import enum
import runpy
import sys
import time
import types
from pathlib import Path

from ..remote_lib.remote_embodiment import RemoteEmbodiment
from ..remote_lib.remote_picarx import RemotePicarx


def _install_python_compatibility_shims() -> None:
    # Python 3.10 does not provide enum.StrEnum, but some SunFounder voice
    # assistant packages import it unconditionally.
    if not hasattr(enum, 'StrEnum'):
        class StrEnum(str, enum.Enum):
            pass

        enum.StrEnum = StrEnum


def _install_picarx_monkey_patch() -> None:
    repo_root = Path(__file__).resolve().parents[5]
    picarx_source_dir = repo_root / 'picarx'

    picarx = types.ModuleType('picarx')
    picarx.__file__ = str(picarx_source_dir / '__init__.py')
    picarx.__package__ = 'picarx'
    picarx.__path__ = [str(picarx_source_dir)]
    sys.modules['picarx'] = picarx

    setattr(picarx, 'Picarx', RemotePicarx)

    picarx_picarx = types.ModuleType('picarx.picarx')
    setattr(picarx_picarx, 'Picarx', RemotePicarx)
    sys.modules['picarx.picarx'] = picarx_picarx

    # Avoid hardware reset calls on the remote-control side.
    picarx_utils = types.ModuleType('picarx.utils')

    def _noop_reset_mcu():
        return

    setattr(picarx_utils, 'reset_mcu', _noop_reset_mcu)
    setattr(picarx, 'utils', picarx_utils)
    sys.modules['picarx.utils'] = picarx_utils

    picarx_tts = types.ModuleType('picarx.tts')

    def _get_embodiment() -> RemoteEmbodiment:
        # Create the embodiment publisher only for examples that actually use
        # TTS, so movement/sensor-only examples keep the previous behavior.
        return RemoteEmbodiment()

    class _RemoteEspeak:
        def say(self, words: str) -> None:
            _get_embodiment().say_text(words, engine='espeak')

    class _RemotePico2Wave:
        def __init__(self) -> None:
            self._lang = 'en-US'

        def set_lang(self, lang: str) -> None:
            self._lang = str(lang)

        def say(self, words: str) -> None:
            del self._lang
            _get_embodiment().say_text(words, engine='pico2wave')

    setattr(picarx_tts, 'Espeak', _RemoteEspeak)
    setattr(picarx_tts, 'Pico2Wave', _RemotePico2Wave)
    setattr(picarx, 'tts', picarx_tts)
    sys.modules['picarx.tts'] = picarx_tts


def main() -> None:
    parser = argparse.ArgumentParser(description='Run a PI-CAR-X example with ROS remote car access.')
    parser.add_argument('script', help='Path to the example script to execute.')
    parser.add_argument('script_args', nargs=argparse.REMAINDER, help='Arguments forwarded to the script.')
    args = parser.parse_args()

    _install_python_compatibility_shims()
    _install_picarx_monkey_patch()

    script_name = Path(args.script).name
    if script_name == 'servo_zeroing.py':
        car = RemotePicarx()
        car.reset()
        # This is a one-shot command. Keep the process alive briefly so the
        # steering/pan/tilt zero commands have time to reach the PI before exit.
        time.sleep(0.5)
        return

    sys.argv = [args.script] + list(args.script_args)
    runpy.run_path(args.script, run_name='__main__')


if __name__ == '__main__':
    main()
