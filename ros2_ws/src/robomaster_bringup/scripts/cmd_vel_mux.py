#!/usr/bin/env python3
"""Entry point for `ros2 run robomaster_bringup cmd_vel_mux.py`."""
import sys

from robomaster_bringup.cmd_vel_mux import main


if __name__ == "__main__":
    sys.exit(main())
