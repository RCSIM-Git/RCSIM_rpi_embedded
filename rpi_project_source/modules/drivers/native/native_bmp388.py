"""
Natywny sterownik dla barometru BMP388 używający biblioteki Adafruit CircuitPython.
Native driver for BMP388 Barometer using Adafruit CircuitPython library.
"""

import logging
import time
from typing import Any

try:
    import adafruit_bmp3xx
    import board
    import busio

    AVAILABLE: bool = True
except ImportError:
    AVAILABLE = False

logger: logging.Logger = logging.getLogger(__name__)


class NativeBMP388:
    """
    Sterownik dla sensora ciśnienia i temperatury BMP388.
    Driver for the BMP388 pressure and temperature sensor.
    """

    def __init__(self, i2c_wrapper: Any | None = None, address: int = 0x77) -> None:
        """
        Inicjalizuje sensor BMP388 przez magistralę I2C.
        Initializes the BMP388 sensor via I2C.

        Args:
            i2c_wrapper (Any | None): Wrapper I2C (opcjonalny, używa busio).
                                         I2C wrapper (optional, uses busio).
            address (int): Adres I2C urządzenia (0x77 lub 0x76). / I2C address.
        """
        if not AVAILABLE:
            raise ImportError("adafruit-circuitpython-bmp3xx library not found")

        try:
            self.i2c: busio.I2C = busio.I2C(board.SCL, board.SDA)
            # Domyślny adres BMP388 to 0x77. Może też być 0x76.
            self.bmp: adafruit_bmp3xx.BMP3XX_I2C = adafruit_bmp3xx.BMP3XX_I2C(
                self.i2c, address=address
            )

            self.bmp.pressure_oversampling = 8
            self.bmp.temperature_oversampling = 2

            logger.info(f"BMP388 initialized at {hex(address)}")

            # Stan mechanizmu zabezpieczającego (Circuit Breaker)
            self.error_count: int = 0
            self.is_disabled: bool = False
            self.disabled_until: float = 0.0

        except Exception as e:
            logger.debug(f"Failed to init BMP388 at {hex(address)}: {e}")
            self.bmp = None

    def read_data(self) -> dict[str, Any]:
        """
        Odczytuje dane z sensora i zwraca je w formacie słownika.
        Reads data from the sensor and returns it in dictionary format.

        Returns:
            dict[str, Any]: Słownik zawierający 'pressure' i 'temperature'.
                            Dictionary containing 'pressure' and 'temperature'.
        """
        if self.bmp is None:
            return {}

        now: float = time.time()
        if self.is_disabled:
            if now > self.disabled_until:
                self.is_disabled = False
                self.error_count = 0
            else:
                return {}

        try:
            p: float = self.bmp.pressure
            t: float = self.bmp.temperature
            # Sukces - zresetuj licznik błędów
            self.error_count = 0
            return {
                "pressure": p * 100.0,  # hPa na Pa
                "temperature": t,
            }
        except Exception as e:
            self.error_count += 1
            if self.error_count < 5:
                logger.warning(f"BMP388 read error: {e}")
            elif self.error_count == 5:
                logger.warning("BMP388: Too many errors. Disabling for 5 seconds.")

            # Wyłącz sensor po 10 kolejnych błędach
            # Circuit break after 10 consecutive errors
            if self.error_count > 10:
                self.is_disabled = True
                self.disabled_until = now + 5.0

            return {}
