#!/usr/bin/env python3
"""
Diagnostic script to check available serial ports and video devices on Raspberry Pi.
Run this inside the container or on the host to see what's available.
"""

import glob
import os


def check_devices():
    print("=== Serial Ports (/dev/tty* and /dev/serial*) ===")

    # Check standard serial aliases
    aliases = glob.glob("/dev/serial*")
    for alias in aliases:
        try:
            target = os.readlink(alias)
            print(f"  {alias} -> {target}")
        except OSError:
            print(f"  {alias}")

    # Check TTY USB/AMA
    ttys = (
        glob.glob("/dev/ttyUSB*")
        + glob.glob("/dev/ttyAMA*")
        + glob.glob("/dev/ttyACM*")
    )
    for tty in sorted(ttys):
        print(f"  {tty}")

    if not aliases and not ttys:
        print("  No serial ports found!")

    print("\n=== Video Devices (/dev/video*) ===")
    videos = glob.glob("/dev/video*")
    for video in sorted(videos):
        print(f"  {video}")

    if not videos:
        print("  No video devices found!")

    print("\n=== I2C Devices (/dev/i2c*) ===")
    i2cs = glob.glob("/dev/i2c*")
    for i2c in sorted(i2cs):
        print(f"  {i2c}")


if __name__ == "__main__":
    check_devices()
