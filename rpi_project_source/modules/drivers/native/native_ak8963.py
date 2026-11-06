"""PL: Odczytuje trójosiowy magnetometr AK8963 przez wspólny wrapper I2C.

EN: Reads the three-axis AK8963 magnetometer through a shared I2C wrapper.
"""

import logging
import time

from .native_i2c import I2CWrapper


class NativeAK8963:
    """PL: Konfiguruje AK8963 i udostępnia skorygowane pomiary pola.

    EN: Configures the AK8963 and provides corrected magnetic field readings.
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
        """PL: Weryfikuje identyfikator układu, kalibrację i tryb pomiaru.

        Args:
            i2c_wrapper: Współdzielony wrapper magistrali I2C.
            address: Adres AK8963, domyślnie 0x0C.

        EN: Verifies the chip identifier, loads sensitivity data, and sets the mode.

        Args:
            i2c_wrapper: Shared I2C bus wrapper.
            address: AK8963 address, defaulting to 0x0C.
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
        """PL: Odczytuje fabryczne współczynniki czułości z Fuse ROM.

        Returns:
            Słownik współczynników korekcyjnych osi x, y i z.

        EN: Reads factory sensitivity adjustment values from Fuse ROM.

        Returns:
            A dictionary of adjustment coefficients for the x, y, and z axes.
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
        """PL: Ustawia tryb pomiaru i rozdzielczość wyjścia magnetometru.

        Args:
            mode: Wartość trybu pracy układu.
            bit_output: 0 wybiera wyjście 14-bitowe, 1 — 16-bitowe.

        EN: Sets the magnetometer measurement mode and output resolution.

        Args:
            mode: Sensor operating mode value.
            bit_output: 0 selects 14-bit output; 1 selects 16-bit output.
        """
        # 🔧 FIX: Zgodność z kodem ricardozago - dodaj bit_output
        # GitHub: 0b00010110 = continuous mode 2 (100Hz) + 16-bit output
        # Bit 4: 0=14bit, 1=16bit | Bits 3-0: mode
        self._bit_output = bit_output
        control_value = mode | (bit_output << 4)
        self.i2c.write_byte_data(self.address, self._CNTL1, control_value)
        time.sleep(0.01)

    def _bytes_to_int(self, low: int, high: int) -> int:
        """PL: Dekoduje dwa bajty little-endian jako liczbę 16-bit ze znakiem.

        EN: Decodes two little-endian bytes as a signed 16-bit integer.
        """
        value = (high << 8) | low
        return value if value < 32768 else value - 65536

    def read_scaled(self) -> tuple[float, float, float]:
        """PL: Odczytuje, skaluje i koryguje wektor pola magnetycznego w µT.

        Returns:
            Wartości osi x, y i z albo NaN przy przepełnieniu pomiaru.

        EN: Reads, scales, and corrects the magnetic field vector in µT.

        Returns:
            x, y, and z values, or NaN values when the measurement overflows.
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
        """PL: Zwraca pomiary osi magnetometru jako słownik dla HardwareManager.

        EN: Returns magnetometer axis readings as a HardwareManager-compatible dictionary.
        """
        mx, my, mz = self.read_scaled()
        return {"mx": mx, "my": my, "mz": mz}
