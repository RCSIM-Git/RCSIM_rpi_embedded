"""
Native AK8963 Magnetometer Driver using a shared I2C wrapper.
Natywny sterownik magnetometru AK8963 używający współdzielonego wrappera I2C.

This driver provides a low-level interface to the AK8963 3-axis magnetometer,
which is commonly found integrated into the MPU-9250 IMU.

Ten sterownik dostarcza niskopoziomowy interfejs do 3-osiowego magnetometru
AK8963, który jest często zintegrowany w IMU MPU-9250.
"""

import logging
import time

from .native_i2c import I2CWrapper


class NativeAK8963:
    """
    A lightweight driver for the AK8963 magnetometer.
    Lekki sterownik dla magnetometru AK8963.
    """

    _DEFAULT_ADDRESS = 0x0C

    # --- Register Definitions ---
    _WIA = 0x00  # Who I Am register
    _HXL = 0x03  # Start of magnetometer data registers
    _ST2 = 0x09  # Status 2 register, indicates magnetic overflow
    _CNTL1 = 0x0A  # Control 1 register
    _CNTL2 = 0x0B  # Control 2 register
    _ASAX = 0x10  # Start of sensitivity adjustment registers

    # --- Mode Definitions for CNTL1 ---
    _MODE_POWER_DOWN = 0x00
    _MODE_CONTINUOUS_1 = 0x02  # 8Hz continuous measurement
    _MODE_CONTINUOUS_2 = 0x06  # 100Hz continuous measurement
    _MODE_FUSE_ROM = 0x0F  # Fuse ROM access mode

    _WHO_AM_I_RESPONSE = 0x48  # Expected value from WIA register

    def __init__(
        self, i2c_wrapper: I2CWrapper, address: int = _DEFAULT_ADDRESS
    ) -> None:
        """
        Initializes the AK8963 driver.
        Inicjalizuje sterownik AK8963.

        Args:
            i2c_wrapper (I2CWrapper): The shared I2C wrapper instance.
            address (int): The I2C address of the AK8963. Defaults to 0x0C.
        """
        self.i2c: I2CWrapper = i2c_wrapper
        self.address: int = address

        chip_id: int = self.i2c.read_byte_data(self.address, self._WIA)
        if chip_id != self._WHO_AM_I_RESPONSE:
            raise RuntimeError(
                f"Invalid AK8963 chip ID: expected 0x48, got {hex(chip_id)}"
            )

        self._sensitivity: dict[str, float] = self._read_sensitivity_adjustments()
        self._bit_output: int = 1  # Domyślnie 16-bit
        self.set_mode()
        logging.info("✓ Detected AK8963 magnetometer")

    def _read_sensitivity_adjustments(self) -> dict[str, float]:
        """
        Reads the factory-programmed sensitivity adjustment values from Fuse ROM.
        Odczytuje fabrycznie zaprogramowane wartości korekcji czułości z Fuse ROM.

        These values are used to correct the raw magnetometer data.
        Te wartości są używane do korekcji surowych danych magnetometru.

        Returns:
            dict[str, float]: A dictionary with the adjustment values for each axis.
        """
        # To read the Fuse ROM, we must first enter Fuse ROM access mode
        # Aby odczytać Fuse ROM, musimy najpierw wejść w tryb dostępu do Fuse ROM
        self.i2c.write_byte_data(self.address, self._CNTL1, self._MODE_FUSE_ROM)
        time.sleep(0.01)

        rom_data = self.i2c.read_i2c_block_data(self.address, self._ASAX, 3)

        # Power down the device to exit Fuse ROM mode
        # Wyłącz zasilanie urządzenia, aby wyjść z trybu Fuse ROM
        self.i2c.write_byte_data(self.address, self._CNTL1, self._MODE_POWER_DOWN)
        time.sleep(0.01)

        # Formula from the datasheet to calculate sensitivity adjustment
        # Wzór z noty katalogowej do obliczenia korekcji czułości
        # 🔧 FIX: Zgodność z kodem ricardozago GY91-MPU9250-BMP280
        # GitHub: (value - 128)/256. + 1.
        return {
            "x": (rom_data[0] - 128) / 256.0 + 1.0,
            "y": (rom_data[1] - 128) / 256.0 + 1.0,
            "z": (rom_data[2] - 128) / 256.0 + 1.0,
        }

    def set_mode(self, mode: int = _MODE_CONTINUOUS_2, bit_output: int = 1) -> None:
        """
        Sets the measurement mode of the magnetometer.
        Ustawia tryb pomiaru magnetometru.

        Args:
            mode (int): The desired mode (e.g., `_MODE_CONTINUOUS_2` for 100Hz).
            bit_output (int): Output bit setting: 0 for 14-bit, 1 for 16-bit (default).
                             Ustawienie bitów wyjściowych: 0 dla 14-bit, 1 dla 16-bit.
        """
        # 🔧 FIX: Zgodność z kodem ricardozago - dodaj bit_output
        # GitHub: 0b00010110 = continuous mode 2 (100Hz) + 16-bit output
        # Bit 4: 0=14bit, 1=16bit | Bits 3-0: mode
        self._bit_output = bit_output
        control_value = mode | (bit_output << 4)
        self.i2c.write_byte_data(self.address, self._CNTL1, control_value)
        time.sleep(0.01)

    def _bytes_to_int(self, low: int, high: int) -> int:
        """Converts two bytes to a signed 16-bit integer (little-endian)."""
        value = (high << 8) | low
        return value if value < 32768 else value - 65536

    def read_scaled(self) -> tuple[float, float, float]:
        """
        Reads, scales, and adjusts the magnetometer data.
        Odczytuje, skaluje i koryguje dane magnetometru.

        Returns:
            tuple[float, float, float]: Corrected x, y, z magnetic field data in µT.
                                        Krotka ze skorygowanymi danymi x, y, z w µT.
        """
        # Read the 7-byte block containing data and the status register
        # Odczytaj 7-bajtowy blok zawierający dane i rejestr statusu
        data = self.i2c.read_i2c_block_data(self.address, self._HXL, 7)

        # Check for magnetic sensor overflow
        # Sprawdź przepełnienie czujnika magnetycznego
        if data[6] & 0x08:
            # Overflow occurred, data is unreliable
            # Wystąpiło przepełnienie, dane są niewiarygodne
            return float("nan"), float("nan"), float("nan")

        raw_x = self._bytes_to_int(data[0], data[1])
        raw_y = self._bytes_to_int(data[2], data[3])
        raw_z = self._bytes_to_int(data[4], data[5])

        # The resolution depends on the bit_output setting (14-bit or 16-bit)
        # Rozdzielczość zależy od ustawienia bit_output (14-bit lub 16-bit)
        if self._bit_output == 0:  # 14-bit
            # 14-bit: 4912 µT range for +/-8190 raw value (~0.6 µT/LSB)
            resolution = 4912 / 8190.0
        else:  # 16-bit (default)
            # 16-bit: 4912 µT range for +/-32760 raw value (~0.15 µT/LSB)
            resolution = 4912 / 32760.0

        scaled_x = raw_x * self._sensitivity["x"] * resolution
        scaled_y = raw_y * self._sensitivity["y"] * resolution
        scaled_z = raw_z * self._sensitivity["z"] * resolution

        return scaled_x, scaled_y, scaled_z

    def read_data(self) -> dict[str, float]:
        """
        Returns data in a dictionary format for HardwareManager compatibility.
        Zwraca dane w formacie słownika dla kompatybilności z HardwareManager.
        """
        mx, my, mz = self.read_scaled()
        return {"mx": mx, "my": my, "mz": mz}
