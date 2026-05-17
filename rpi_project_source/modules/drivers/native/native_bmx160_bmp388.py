"""
Połączony sterownik dla modułu 10-DoF (BMX160 + BMP388).
Combined driver for 10-DoF module (BMX160 + BMP388).
"""

import logging
from typing import Any

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

from .native_bmp388 import NativeBMP388
from .native_bmx160 import NativeBMX160
from .native_i2c import I2CWrapper

logger: logging.Logger = logging.getLogger(__name__)


@SensorRegistry.register
class NativeBMX160BMP388(IMUBase):
    """
    Sterownik obsługujący kaskadowe połączenie BMX160 i BMP388 na jednej magistrali.
    Driver handling the cascaded connection of BMX160 and BMP388 on a single bus.
    """

    DRIVER_NAME = "native_bmx160_bmp388"
    I2C_ADDRESSES = [0x68, 0x69]
    PRIORITY = 35

    @classmethod
    def scan(cls, i2c) -> bool:
        """Check for BMX160 (0xD1/0xD8) + BMP388."""
        try:
            chip = i2c.read_byte_data(0x68, 0x00)
            if chip not in [0xD1, 0xD8]:
                return False
            # BMP388 present at 0x77 or 0x76?
            for addr in [0x77, 0x76]:
                try:
                    bmp_id = i2c.read_byte_data(addr, 0x00)
                    if bmp_id == 0x50:  # BMP388
                        return True
                except (OSError, IOError):
                    continue
            return False
        except (OSError, IOError):
            return False

    def __init__(self, i2c_wrapper: I2CWrapper) -> None:
        """
        Inicjalizuje oba sensory.
        Initializes both sensors.

        Args:
            i2c_wrapper (I2CWrapper): Wrapper magistrali I2C. / I2C bus wrapper.
        """
        self.bmx: NativeBMX160 | None = None
        self.bmp: NativeBMP388 | None = None

        # BMX160: Try address 0x68 first (AD0 low), then 0x69 (AD0 high)
        try:
            self.bmx = NativeBMX160(i2c_wrapper, address=0x68)
        except Exception as e:
            logger.warning(f"BMX160 init at 0x68 failed: {e}. Trying 0x69...")
            try:
                self.bmx = NativeBMX160(i2c_wrapper, address=0x69)
                logger.info("✓ BMX160 initialized at 0x69")
            except Exception as e2:
                logger.error(f"BMX160 init failed at both 0x68 and 0x69: {e2}")

        # BMP388: Try address 0x76 first, then 0x77
        try:
            self.bmp = NativeBMP388(i2c_wrapper, address=0x76)
            # Sprawdź czy faktycznie się połączył / Check if it actually connected
            if not self.bmp.bmp:
                raise RuntimeError("No device at 0x76")
        except Exception as e:
            logger.debug(f"BMP388 (0x76) failed: {e}. Trying 0x77...")
            try:
                self.bmp = NativeBMP388(i2c_wrapper, address=0x77)
                if not self.bmp.bmp:
                    self.bmp = None
                    logger.warning("BMP388 (0x77) also failed (Optional sensor).")
            except Exception as e2:
                self.bmp = None
                logger.debug(f"BMP388 init failed completely: {e2}")

    def read_data(self) -> dict[str, Any]:
        """
        Odczytuje dane z BMX160 i BMP388, łącząc je w jeden słownik.
        Reads data from BMX160 and BMP388, merging them into a single dictionary.

        Returns:
            dict[str, Any]: Połączone dane 10-DoF. / Merged 10-DoF data.
        """
        data: dict[str, Any] = {}
        if self.bmx:
            data.update(self.bmx.read_data())
        if self.bmp:
            data.update(self.bmp.read_data())
        return data

    def calibrate(self) -> bool:
        """
        Przeprowadza kalibrację sensora BMX160.
        Performs calibration of the BMX160 sensor.
        """
        if self.bmx:
            return self.bmx.calibrate()
        return False
