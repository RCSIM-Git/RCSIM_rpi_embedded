#!/usr/bin/env python3
"""
Diagnostic script for GY-91 (MPU9250 + BMP280) Magnetometer issues.
Diagnostyka problemów z magnetometrem GY-91 (MPU9250 + BMP280).

Usage:
    python3 diagnose_imu.py
"""

import sys
import threading
import time

# --- Embedded I2CWrapper Class for Standalone Execution ---
try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus
    except ImportError:
        print(
            "CRITICAL: smbus2 or smbus not found. Please install it: sudo apt-get install python3-smbus"
        )
        sys.exit(1)


class I2CWrapper:
    """
    A wrapper for the smbus2.SMBus class to handle I2C communication.
    Wrapper dla klasy smbus2.SMBus do obsługi komunikacji I2C.
    """

    def __init__(self, bus_num: int = 1):
        self.bus = SMBus(bus_num)
        self._lock = threading.Lock()

    def read_byte_data(self, address: int, register: int) -> int:
        with self._lock:
            return self.bus.read_byte_data(address, register)

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        with self._lock:
            self.bus.write_byte_data(address, register, value)

    def read_word_data(self, address: int, register: int) -> int:
        with self._lock:
            return self.bus.read_word_data(address, register)

    def read_i2c_block_data(self, address: int, register: int, length: int) -> list:
        with self._lock:
            return self.bus.read_i2c_block_data(address, register, length)

    def write_i2c_block_data(self, address: int, register: int, data: list) -> None:
        with self._lock:
            self.bus.write_i2c_block_data(address, register, data)

    def close(self) -> None:
        with self._lock:
            self.bus.close()


# --- End of Embedded I2CWrapper ---

# register definitions
MPU_ADDRESS = 0x68
REG_WHO_AM_I = 0x75
REG_PWR_MGMT_1 = 0x6B
REG_INT_PIN_CFG = 0x37
REG_USER_CTRL = 0x6A

AK8963_ADDRESS = 0x0C
AK_WHO_AM_I = 0x00


def scan_i2c(i2c, label="I2C Scan"):
    """
    Skanuje magistralę I2C.
    Scans the I2C bus.
    """
    print(f"\n--- {label} ---")
    found = []
    for addr in range(0x03, 0x78):
        try:
            i2c.bus.write_quick(addr)
            found.append(addr)
        except Exception:
            pass

    if found:
        print(f"Devices found: {[hex(x) for x in found]}")
    else:
        print("No devices found!")
    return found


def main():
    print("Initializing I2C Bus 1...")
    try:
        i2c = I2CWrapper(bus_num=1)
    except Exception as e:
        print(f"CRITICAL: Failed to open I2C bus: {e}")
        return

    # 1. Initial Scan
    devices = scan_i2c(i2c, "Initial Bus Scan")

    if MPU_ADDRESS not in devices:
        print(f"CRITICAL: MPU-9250 not found at 0x{MPU_ADDRESS:02X}!")
        return

    # 2. Check MPU WhoAmI
    try:
        who_am_i = i2c.read_byte_data(MPU_ADDRESS, REG_WHO_AM_I)
        print(f"\nMPU WHO_AM_I (0x75): 0x{who_am_i:02X}")

        chip_names = {
            0x71: "MPU-9250",
            0x73: "MPU-9255",
            0x70: "MPU-6500",
            0x68: "MPU-6050",
        }
        print(f"Identified as: {chip_names.get(who_am_i, 'Unknown')}")

    except Exception as e:
        print(f"Error reading MPU WHO_AM_I: {e}")
        return

    # 3. Reset MPU
    print("\nResetting MPU...")
    i2c.write_byte_data(MPU_ADDRESS, REG_PWR_MGMT_1, 0x80)
    time.sleep(0.1)

    # Wake up
    i2c.write_byte_data(MPU_ADDRESS, REG_PWR_MGMT_1, 0x01)
    time.sleep(0.1)

    # 4. Configure Bypass
    print("\nConfiguring Bypass Mode...")

    # Disable I2C Master Mode if enabled (Bit 4 of USER_CTRL)
    user_ctrl = i2c.read_byte_data(MPU_ADDRESS, REG_USER_CTRL)
    print(f"Current USER_CTRL (0x6A): 0x{user_ctrl:02X}")
    if user_ctrl & 0x20:
        print("I2C Master Mode is enabled. Disabling it...")
        i2c.write_byte_data(MPU_ADDRESS, REG_USER_CTRL, user_ctrl & ~0x20)
        time.sleep(0.01)

    # Enable Bypass (Bit 1 of INT_PIN_CFG)
    int_pin_cfg = i2c.read_byte_data(MPU_ADDRESS, REG_INT_PIN_CFG)
    print(f"Current INT_PIN_CFG (0x37): 0x{int_pin_cfg:02X}")

    print("Setting INT_PIN_CFG to 0x02 (logic OR)...")
    i2c.write_byte_data(MPU_ADDRESS, REG_INT_PIN_CFG, int_pin_cfg | 0x02)
    time.sleep(0.05)

    new_int_pin_cfg = i2c.read_byte_data(MPU_ADDRESS, REG_INT_PIN_CFG)
    print(f"New INT_PIN_CFG (0x37): 0x{new_int_pin_cfg:02X}")

    if not (new_int_pin_cfg & 0x02):
        print("WARNING: Failed to set Bypass Bit!")
    else:
        print("Bypass Bit SET.")

    # 5. Scan Again
    devices_after = scan_i2c(i2c, "Bus Scan AFTER Bypass Enable")

    if AK8963_ADDRESS in devices_after:
        print(f"\nSUCCESS! Magnetometer (AK8963) detected at 0x{AK8963_ADDRESS:02X}")
        try:
            ak_who = i2c.read_byte_data(AK8963_ADDRESS, AK_WHO_AM_I)
            print(f"AK8963 WHO_AM_I (0x00): 0x{ak_who:02X} (Expected: 0x48)")
        except Exception as e:
            print(f"Error reading AK8963 WHO_AM_I: {e}")
    else:
        print(f"\nFAILURE: Magnetometer (0x{AK8963_ADDRESS:02X}) NOT detected.")
        print("Possible causes:")
        print(
            "1. Fake MPU-9250 (actually MPU-6500) with no internal magnetometer connection."
        )
        print("2. Bypass mode failed to engage physically.")
        print("3. Defective GY-91 module.")

    i2c.close()


if __name__ == "__main__":
    main()
