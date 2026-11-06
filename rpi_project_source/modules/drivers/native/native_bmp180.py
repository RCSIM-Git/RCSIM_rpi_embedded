# The MIT License (MIT)
# Copyright (c) 2024 Jules
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
# OR OTHER DEALINGS IN THE SOFTWARE.

"""PL: Odczytuje BMP180 i przelicza surowe próbki na ciśnienie oraz temperaturę.

Sterownik pobiera dane kalibracyjne i stosuje wzory kompensacyjne Boscha.

EN: Reads the BMP180 and converts raw samples into pressure and temperature.

The driver loads calibration data and applies Bosch compensation formulas.
"""

import logging
import struct
import time

from .native_i2c import I2CWrapper


class NativeBMP180:
    """PL: Obsługuje BMP180 i oblicza pomiary według wzorów z noty katalogowej.

    EN: Operates the BMP180 and calculates readings using the datasheet formulas.
    """

    # --- I2C and Register Definitions ---
    _DEFAULT_ADDRESS = 0x77
    _REG_CALIB_START = 0xAA  # Start of 22-byte calibration data block in EEPROM
    _REG_CONTROL = 0xF4  # Control register for measurements
    _REG_DATA_START = 0xF6  # Start of measurement data registers (MSB, LSB, XLSB)

    # --- Commands for the Control Register ---
    _CMD_READ_TEMP = 0x2E  # Command to start a temperature measurement
    _CMD_READ_PRESSURE = 0x34  # Command to start a pressure measurement (standard mode)

    _REG_CHIP_ID = 0xD0
    _CHIP_ID = 0x55

    def __init__(self, i2c_wrapper: I2CWrapper, address: int = _DEFAULT_ADDRESS):
        """PL: Weryfikuje identyfikator BMP180 i wczytuje współczynniki kalibracji.

        Args:
            i2c_wrapper: Współdzielony wrapper magistrali I2C.
            address: Adres I2C czujnika BMP180, domyślnie 0x77.

        EN: Verifies the BMP180 identifier and loads calibration coefficients.

        Args:
            i2c_wrapper: Shared I2C bus wrapper.
            address: BMP180 I2C address, defaulting to 0x77.
        """
        self.i2c = i2c_wrapper
        self.address = address

        # Verify chip ID
        chip_id = self.i2c.read_byte_data(self.address, self._REG_CHIP_ID)
        if chip_id != self._CHIP_ID:
            raise RuntimeError(
                f"Invalid BMP180 chip ID: expected {hex(self._CHIP_ID)}, got {hex(chip_id)}"
            )
        logging.info(f"✓ Detected BMP180 at address {hex(self.address)}")

        # Store calibration coefficients in a dictionary for clarity
        # Przechowuj współczynniki kalibracyjne w słowniku dla czytelności
        self.calib_coeffs: dict[str, int] = {}
        self._load_calibration_data()

    def _load_calibration_data(self) -> None:
        """PL: Wczytuje 22 bajty fabrycznych współczynników kalibracyjnych.

        EN: Reads the 22 bytes of factory calibration coefficients.
        """
        calib_data = self.i2c.read_i2c_block_data(
            self.address, self._REG_CALIB_START, 22
        )

        # The format string '>hhhHHHhhhhh' means:
        # >: big-endian
        # h: signed short (2 bytes)
        # H: unsigned short (2 bytes)
        coeffs = struct.unpack(">hhhHHHhhhhh", bytes(calib_data))

        keys = ["ac1", "ac2", "ac3", "ac4", "ac5", "ac6", "b1", "b2", "mb", "mc", "md"]
        self.calib_coeffs = dict(zip(keys, coeffs))

    def _read_raw_temp(self) -> int:
        """PL: Uruchamia pomiar i odczytuje surową wartość temperatury.

        Returns:
            Surową 16-bitową wartość temperatury.

        EN: Starts a measurement and reads the raw temperature value.

        Returns:
            The raw 16-bit temperature value.
        """
        self.i2c.write_byte_data(self.address, self._REG_CONTROL, self._CMD_READ_TEMP)
        time.sleep(0.005)  # Datasheet specifies a max 4.5ms wait time
        data = self.i2c.read_i2c_block_data(self.address, self._REG_DATA_START, 2)
        return (data[0] << 8) | data[1]

    def _read_raw_pressure(self) -> int:
        """PL: Uruchamia pomiar i odczytuje surową wartość ciśnienia.

        Returns:
            Surową wartość ciśnienia przed kompensacją.

        EN: Starts a measurement and reads the raw pressure value.

        Returns:
            The uncompensated pressure value.
        """
        # OSS=0 (oversampling setting)
        self.i2c.write_byte_data(
            self.address, self._REG_CONTROL, self._CMD_READ_PRESSURE
        )
        time.sleep(0.008)  # Datasheet: max 7.5ms for standard mode
        data = self.i2c.read_i2c_block_data(self.address, self._REG_DATA_START, 3)
        # The value is 16 to 19 bits long; we shift out the unused bits
        # Wartość ma długość od 16 do 19 bitów; przesuwamy, aby odrzucić nieużywane bity
        return ((data[0] << 16) + (data[1] << 8) + data[2]) >> 8

    def read_scaled(self) -> tuple[float, float]:
        """PL: Oblicza skompensowaną temperaturę w °C i ciśnienie w paskalach.

        Returns:
            Krotkę zawierającą temperaturę i ciśnienie.

        EN: Calculates compensated temperature in °C and pressure in pascals.

        Returns:
            A tuple containing temperature and pressure.
        """
        c = self.calib_coeffs  # Use a shorter alias for convenience

        # --- Temperature Calculation ---
        # These steps are a direct implementation of the datasheet's algorithm.
        # Poniższe kroki są bezpośrednią implementacją algorytmu z noty katalogowej.
        ut = self._read_raw_temp()
        x1_t = ((ut - c["ac6"]) * c["ac5"]) >> 15
        x2_t = (c["mc"] << 11) // (x1_t + c["md"])
        b5 = x1_t + x2_t
        temp = ((b5 + 8) >> 4) / 10.0

        # --- Pressure Calculation ---
        up = self._read_raw_pressure()
        b6 = b5 - 4000
        x1_p = (c["b2"] * ((b6 * b6) >> 12)) >> 11
        x2_p = (c["ac2"] * b6) >> 11
        x3_p = x1_p + x2_p
        # Formula depends on oversampling setting (OSS), here we assume OSS=0
        # Wzór zależy od ustawienia oversamplingu (OSS), tutaj zakładamy OSS=0
        b3 = ((c["ac1"] * 4 + x3_p) + 2) // 4

        x1_p = (c["ac3"] * b6) >> 13
        x2_p = (c["b1"] * ((b6 * b6) >> 12)) >> 16
        x3_p = ((x1_p + x2_p) + 2) >> 2
        b4 = (c["ac4"] * (x3_p + 32768)) >> 15
        b7 = (up - b3) * 50000

        if b7 < 0x80000000:
            p = (b7 * 2) // b4
        else:
            p = (b7 // b4) * 2

        x1_p = (p >> 8) * (p >> 8)
        x1_p = (x1_p * 3038) >> 16
        x2_p = (-7357 * p) >> 16

        pressure = p + ((x1_p + x2_p + 3791) >> 4)

        return temp, pressure

    def read_data(self) -> dict[str, float]:
        """PL: Zwraca temperaturę i ciśnienie jako słownik zgodny z HardwareManager.

        EN: Returns temperature and pressure in a HardwareManager-compatible dictionary.
        """
        temp, pressure = self.read_scaled()
        return {"temperature": temp, "pressure": pressure}
