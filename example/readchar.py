"""Small compatibility shim for examples that expect the third-party readchar package."""

from __future__ import annotations

import sys
import termios
import tty


class key:
    SPACE = ' '
    CTRL_C = '\x03'
    ESC = ('\x1b',)


def readchar() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def readkey() -> str:
    return readchar()
