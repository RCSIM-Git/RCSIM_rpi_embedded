"""PL: Łączy odczyty IMU BMX160 i barometru BMP388.

EN: Combines readings from the BMX160 IMU and BMP388 barometer.
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
    """PL: Obsługuje BMX160 i BMP388 na współdzielonej magistrali I2C.

    EN: Operates the BMX160 and BMP388 on a shared I2C bus.
    """

    DRIVER_NAME = "native_bmx160_bmp388"
    I2C_ADDRESSES = [0x68, 0x69]
    PRIORITY = 35

    @classmethod
    def scan(cls, i2c) -> bool:
        """PL: Sprawdza identyfikatory BMX160 i BMP388 na magistrali I2C.

        EN: Checks for BMX160 and BMP388 chip identifiers on the I2C bus.
        """
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
        """PL: Inicjalizuje sensory, próbując obsługiwane adresy I2C.

        Args:
            i2c_wrapper: Wrapper współdzielonej magistrali I2C.

        EN: Initializes the sensors, trying their supported I2C addresses.

        Args:
            i2c_wrapper: Wrapper for the shared I2C bus.
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
        """PL: Łączy dostępne pomiary BMX160 i BMP388 w jednym słowniku.

        Returns:
            Słownik z danymi sensorów, które udało się odczytać.

        EN: Merges available BMX160 and BMP388 readings into one dictionary.

        Returns:
            A dictionary containing readings from sensors that were available.
        """
        data: dict[str, Any] = {}
        if self.bmx:
            data.update(self.bmx.read_data())
        if self.bmp:
            data.update(self.bmp.read_data())
        return data

    def calibrate(self) -> bool:
        """PL: Deleguje kalibrację do BMX160, jeśli sensor jest dostępny.

        EN: Delegates calibration to the BMX160 when the sensor is available.
        """
        if self.bmx:
            return self.bmx.calibrate()
        return False
