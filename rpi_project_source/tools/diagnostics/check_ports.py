#!/usr/bin/env python3
"""PL: Pokazuje dostępne porty szeregowe, urządzenia wideo i magistrale I2C.

EN: List available serial ports, video devices, and I2C buses.
"""

import glob
import os


def check_devices():
    """PL: Wypisuje wykryte urządzenia systemowe używane przez RPI.

    EN: Print detected system devices used by the RPI application.
    """
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
