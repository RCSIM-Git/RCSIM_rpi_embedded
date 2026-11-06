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

"""PL: Odczytuje surowe próbki magnetometru QMC5883L przez wspólny wrapper I2C.

EN: Reads raw QMC5883L magnetometer samples through a shared I2C wrapper.
"""

import logging

from .native_i2c import I2CWrapper


class NativeQMC5883L:
    """PL: Steruje QMC5883L bezpośrednio przez rejestry I2C.

    EN: Controls the QMC5883L directly through its I2C registers.
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
        """PL: Ustawia tryb pomiaru i inicjalizuje kompensację magnetometru.

        Args:
            i2c_wrapper: Współdzielony wrapper magistrali I2C.
            address: Adres QMC5883L, domyślnie 0x0D.

        EN: Sets the measurement mode and initializes magnetometer compensation.

        Args:
            i2c_wrapper: Shared I2C bus wrapper.
            address: QMC5883L address, defaulting to 0x0D.
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
        """PL: Łączy dwa bajty little-endian w 16-bitową liczbę ze znakiem.

        Args:
            low: Młodszy bajt wartości.
            high: Starszy bajt wartości.

        Returns:
            Zdekodowaną liczbę całkowitą ze znakiem.

        EN: Combines two little-endian bytes into a signed 16-bit integer.

        Args:
            low: Least significant byte.
            high: Most significant byte.

        Returns:
            The decoded signed integer.
        """
        value: int = (high << 8) | low
        # Two's complement conversion for negative numbers
        # Konwersja z uzupełnienia do dwóch dla liczb ujemnych
        return value if value < 32768 else value - 65536

    def read_scaled(self, throttle: float = 0.0) -> tuple[float, float, float]:
        """PL: Odczytuje osie magnetometru i odejmuje kompensację zależną od gazu.

        Zwraca wartości surowe proporcjonalne do pola; nie są przeliczane na gausy.

        Returns:
            Krotkę surowych wartości osi mx, my i mz.

        EN: Reads magnetometer axes and subtracts throttle-dependent compensation.

        Returns:
            A tuple of raw mx, my, and mz values proportional to the field;
            values are not converted to gauss.
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
        """PL: Zwraca próbki magnetometru w słowniku zgodnym z innymi sensorami.

        Returns:
            Słownik z wartościami mx, my i mz.

        EN: Returns magnetometer samples in a dictionary compatible with other sensors.

        Returns:
            A dictionary containing mx, my, and mz values.
        """
        mx, my, mz = self.read_scaled(throttle=throttle)
        return {"mx": mx, "my": my, "mz": mz}
