"""PL: Udostępnia natywny odczyt ciśnienia i temperatury z BMP388.

EN: Provides native pressure and temperature readings from a BMP388 sensor.
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
    """PL: Obsługuje BMP388 przez bibliotekę Adafruit i magistralę I2C.

    EN: Operates a BMP388 through the Adafruit library and I2C bus.
    """

    def __init__(self, i2c_wrapper: Any | None = None, address: int = 0x77) -> None:
        """PL: Inicjalizuje BMP388 pod wskazanym adresem I2C.

        Args:
            i2c_wrapper: Opcjonalny argument zachowany dla zgodności; magistrala
                jest tworzona lokalnie przez busio.
            address: Adres I2C urządzenia, zwykle 0x77 lub 0x76.

        EN: Initializes the BMP388 at the specified I2C address.

        Args:
            i2c_wrapper: Optional compatibility argument; the bus is created
                locally through busio.
            address: I2C device address, usually 0x77 or 0x76.
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
        """PL: Odczytuje ciśnienie w paskalach i temperaturę w stopniach Celsjusza.

        Returns:
            Słownik z polami pressure i temperature albo pusty słownik przy błędzie.

        EN: Reads pressure in pascals and temperature in degrees Celsius.

        Returns:
            A dictionary with pressure and temperature, or an empty dictionary on error.
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
