#!/usr/bin/env python3
"""Run PI-CAR-X examples with RemotePicarx patched in."""

from __future__ import annotations

import argparse
import runpy
import sys
import types
from pathlib import Path

from ..remote_lib.remote_picarx import RemotePicarx


def _install_picarx_monkey_patch() -> None:
    try:
        import picarx  # type: ignore
    except Exception:
        picarx = types.ModuleType('picarx')
        sys.modules['picarx'] = picarx

    setattr(picarx, 'Picarx', RemotePicarx)

    picarx_picarx = types.ModuleType('picarx.picarx')
    setattr(picarx_picarx, 'Picarx', RemotePicarx)
    sys.modules['picarx.picarx'] = picarx_picarx

    # Avoid hardware reset calls on remote side.
    try:
        import picarx.utils as picarx_utils  # type: ignore

        def _noop_reset_mcu():
            return

        setattr(picarx_utils, 'reset_mcu', _noop_reset_mcu)
    except Exception:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description='Run a PI-CAR-X example with ROS remote car access.')
    parser.add_argument('script', help='Path to the example script to execute.')
    parser.add_argument('script_args', nargs=argparse.REMAINDER, help='Arguments forwarded to the script.')
    args = parser.parse_args()

    _install_picarx_monkey_patch()

    script_name = Path(args.script).name
    if script_name == 'servo_zeroing.py':
        car = RemotePicarx()
        car.reset()
        return

    sys.argv = [args.script] + list(args.script_args)
    runpy.run_path(args.script, run_name='__main__')


if __name__ == '__main__':
    main()
