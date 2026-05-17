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
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
# OR OTHER DEALINGS IN THE SOFTWARE.

"""
Native BMP280 Barometric Pressure and Temperature Sensor Driver.
Natywny sterownik czujnika ciśnienia i temperatury BMP280.
"""

import logging
import struct

from .native_i2c import I2CWrapper


class NativeBMP280:
    """
    A lightweight driver for the BMP280 barometer, implementing datasheet
    calculations.
    Lekki sterownik dla barometru BMP280, implementujący obliczenia z noty
    katalogowej.
    """

    _DEFAULT_ADDRESS = 0x77
    _REG_ID = 0xD0
    _CHIP_ID_BMP280 = 0x58
    _CHIP_ID_BME280 = 0x60
    _REG_CALIB_START = 0x88
    _REG_CTRL_MEAS = 0xF4
    _REG_DATA_START = 0xF7
    _CTRL_MEAS_SETUP = 0x27

    def __init__(self, i2c_wrapper: I2CWrapper, address: int = _DEFAULT_ADDRESS):
        self.i2c = i2c_wrapper
        self.address = address
        chip_id = self.i2c.read_byte_data(self.address, self._REG_ID)
        if chip_id == self._CHIP_ID_BMP280:
            logging.info(f"✓ Detected BMP280 at address {hex(self.address)}")
        elif chip_id == self._CHIP_ID_BME280:
            logging.info(f"✓ Detected BME280 at address {hex(self.address)}")
        else:
            raise RuntimeError(
                f"Invalid BMP280/BME280 chip ID: expected 0x58 or 0x60, got {hex(chip_id)}"
            )
        self.calib_coeffs: dict[str, float] = {}
        self._load_calibration_data()
        self.i2c.write_byte_data(
            self.address, self._REG_CTRL_MEAS, self._CTRL_MEAS_SETUP
        )
        self.t_fine = 0.0

    def _load_calibration_data(self) -> None:
        """
        Loads calibration data from the sensor.
        Ładuje dane kalibracyjne z czujnika.
        """
        calib_data = self.i2c.read_i2c_block_data(
            self.address, self._REG_CALIB_START, 24
        )
        coeffs_int = struct.unpack("<HhhHhhhhhhhh", bytes(calib_data))
        keys = ["T1", "T2", "T3", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9"]
        self.calib_coeffs = {
            f"dig_{key}": float(val) for key, val in zip(keys, coeffs_int)
        }

    def read_scaled(self) -> tuple[float, float]:
        """
        Reads scaled temperature and pressure.
        Odczytuje przeskalowaną temperaturę i ciśnienie.

        Returns:
            tuple[float, float]: Temperature (deg C) and Pressure (Pa).
        """
        data = self.i2c.read_i2c_block_data(self.address, self._REG_DATA_START, 6)
        raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        c = self.calib_coeffs

        # Temperature compensation
        var1_t = (raw_temp / 16384.0 - c["dig_T1"] / 1024.0) * c["dig_T2"]
        var2_t_base = raw_temp / 131072.0 - c["dig_T1"] / 8192.0
        var2_t = (var2_t_base * var2_t_base) * c["dig_T3"]
        self.t_fine = var1_t + var2_t
        temp = self.t_fine / 5120.0

        # Pressure compensation
        var1_p = self.t_fine / 2.0 - 64000.0
        var2_p = var1_p * var1_p * c["dig_P6"] / 32768.0
        var2_p = var2_p + var1_p * c["dig_P5"] * 2.0
        var2_p = var2_p / 4.0 + c["dig_P4"] * 65536.0
        var1_p = (
            c["dig_P3"] * var1_p * var1_p / 524288.0 + c["dig_P2"] * var1_p
        ) / 524288.0
        var1_p = (1.0 + var1_p / 32768.0) * c["dig_P1"]

        if var1_p == 0:
            return temp, 0.0

        p = 1048576.0 - raw_pressure
        pressure = (p - (var2_p / 4096.0)) * 6250.0 / var1_p
        var1_p = c["dig_P9"] * pressure * pressure / 2147483648.0
        var2_p = pressure * c["dig_P8"] / 32768.0
        pressure = pressure + (var1_p + var2_p + c["dig_P7"]) / 16.0

        return temp, pressure

    def read_data(self) -> dict[str, float]:
        """
        Returns data in a dictionary format for HardwareManager compatibility.
        Zwraca dane w formacie słownika dla kompatybilności z HardwareManager.
        """
        temp, pressure = self.read_scaled()
        return {"temperature": temp, "pressure": pressure}
