#!/usr/bin/env python3
"""Backward-compatible entry point.

This keeps `ros2 run picarx_local_ros2 picarx_node` working by forwarding
to the driver node main function.
"""

from .driver_node import main


if __name__ == "__main__":
    main()
