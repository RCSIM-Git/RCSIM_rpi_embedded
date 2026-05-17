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
Native QMC5883L Magnetometer Driver using a shared I2C wrapper.
Natywny sterownik magnetometru QMC5883L używający współdzielonego wrappera I2C.
"""

import logging

from .native_i2c import I2CWrapper


class NativeQMC5883L:
    """
    A lightweight, direct-register-access driver for the QMC5883L magnetometer.
    Lekki sterownik z bezpośrednim dostępem do rejestrów dla magnetometru
    QMC5883L.
    """

    # --- I2C and Register Definitions ---
    _DEFAULT_ADDRESS = 0x0D
    _REG_X_LSB = 0x00  # Start of the 6-byte data block for X, Y, Z axes
    _REG_CONTROL_1 = 0x09  # Control Register 1 for setting mode, rate, range, etc.
    _REG_CONTROL_2 = 0x0A  # Control Register 2 for reset

    # --- Control Register 1 Configuration (0x1D) ---
    # 0b00011101
    # OSR (Oversample Ratio) = 512 (0b00 << 6) -> Bits 7-6
    # RNG (Full Scale Range) = +/- 8 Gauss (0b01 << 4) -> Bits 5-4
    # ODR (Output Data Rate) = 200Hz (0b11 << 2) -> Bits 3-2
    # MODE (Mode Select) = Continuous (0b01) -> Bits 1-0
    _CONTROL_1_SETUP = 0x1D

    def __init__(
        self, i2c_wrapper: I2CWrapper, address: int = _DEFAULT_ADDRESS
    ) -> None:
        """
        Initializes the QMC5883L driver and sets the measurement mode.
        Inicjalizuje sterownik QMC5883L i ustawia tryb pomiaru.

        Args:
            i2c_wrapper (I2CWrapper): The shared I2C wrapper instance.
            address (int): The I2C address of the QMC5883L. Defaults to 0x0D.
        """
        self.i2c: I2CWrapper = i2c_wrapper
        self.address: int = address
        # Set the desired operational mode for the sensor
        # Ustaw pożądany tryb pracy czujnika
        self.i2c.write_byte_data(
            self.address, self._REG_CONTROL_1, self._CONTROL_1_SETUP
        )
        self.mag_coeff: list[float] = [0.0, 0.0, 0.0]
        logging.info(f"✓ Initialized QMC5883L at address {hex(self.address)}")

    def _bytes_to_int(self, low: int, high: int) -> int:
        """
        Converts two bytes (low and high) to a signed 16-bit integer
        (little-endian).
        Konwertuje dwa bajty (dolny i górny) na 16-bitową liczbę całkowitą
        ze znakiem (little-endian).

        Args:
            low (int): The least significant byte.
            high (int): The most significant byte.

        Returns:
            int: The combined signed 16-bit integer value.
        """
        value: int = (high << 8) | low
        # Two's complement conversion for negative numbers
        # Konwersja z uzupełnienia do dwóch dla liczb ujemnych
        return value if value < 32768 else value - 65536

    def read_scaled(self, throttle: float = 0.0) -> tuple[float, float, float]:
        """
        Reads the raw magnetometer data and returns it.
        Odczytuje surowe dane magnetometru i je zwraca.

        Note: The QMC5883L datasheet does not specify a standard sensitivity or
        scaling factor. The raw integer values are returned, which are
        proportional to the magnetic field strength. Further calibration would
        be needed for absolute measurements in

        Uwaga: Nota katalogowa QMC5883L nie specyfikuje standardowej czułości
        ani współczynnika skalowania. Zwracane są surowe wartości całkowite,
        które są proporcjonalne do natężeniu pola magnetycznego. Dalsza
        kalibracja byłaby wymagana do uzyskania absolutnych pomiarów w
        Gausach.

        Returns:
            tuple[float, float, float]: A tuple containing (mx, my, mz) proportional values.
                                        Krotka zawierająca proporcjonalne wartości (mx, my, mz).
        """
        try:
            # Read 6 data registers starting from X_LSB
            # Odczytaj 6 rejestrów danych, zaczynając od X_LSB
            data: list[int] = self.i2c.read_i2c_block_data(
                self.address, self._REG_X_LSB, 6
            )

            # Data is in Little Endian format (LSB, MSB)
            # Dane są w formacie Little Endian (LSB, MSB)
            mx: float = float(self._bytes_to_int(data[0], data[1]))
            my: float = float(self._bytes_to_int(data[2], data[3]))
            mz: float = float(self._bytes_to_int(data[4], data[5]))

            # [NEW] Apply Magnetic Compensation (CompassMot)
            mx -= throttle * self.mag_coeff[0]
            my -= throttle * self.mag_coeff[1]
            mz -= throttle * self.mag_coeff[2]

            return mx, my, mz
        except Exception as e:
            logging.getLogger(__name__).error(f"QMC5883L read failed: {e}")
            return (0.0, 0.0, 0.0)

    def read_data(self, throttle: float = 0.0) -> dict[str, float]:
        """
        Odczytuje dane i zwraca je w formacie słownika (kompatybilność z innymi sensorami).
        Reads data and returns it in dictionary format (compatibility with other sensors).

        Returns:
            dict[str, float]: Słownik zawierający 'mx', 'my', 'mz'.
                              Dictionary containing 'mx', 'my', 'mz'.
        """
        mx, my, mz = self.read_scaled(throttle=throttle)
        return {"mx": mx, "my": my, "mz": mz}
