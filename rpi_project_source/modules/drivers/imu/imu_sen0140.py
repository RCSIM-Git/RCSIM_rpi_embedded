"""PL: Udostępnia mockowe dane modułu DFRobot SEN0140 do testów.

EN: Provides mock DFRobot SEN0140 module data for testing.

Ten moduł dostarcza symulowane dane dla sensora SEN0140 (ADXL345 + ITG3205 + VCM5883L + BMP280), 
używane do testów systemu bez fizycznego podłączenia sprzętu.
This module provides simulated data for the SEN0140 sensor (ADXL345 + ITG3205 + VCM5883L + BMP280), 
used for testing the system without physical hardware connected.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class SEN0140:
    """PL: Zwraca wyłącznie symulowane pomiary modułu SEN0140.

    EN: Returns simulated readings for the SEN0140 module only.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """PL: Zapisuje magistralę I2C przekazaną do mockowego sterownika.

        EN: Stores the I2C bus passed to the mock driver.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik SEN0140 (mock) zainicjalizowany. / SEN0140 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """PL: Zwraca symulowane dane IMU i barometru w ustandaryzowanym słowniku.

        EN: Returns simulated IMU and barometer data in a standardized dictionary.
        """
        return {
            "ax": 9.81,
            "ay": 0.05,
            "az": 0.15,
            "gx": 0.015,
            "gy": 0.025,
            "gz": 0.035,
            "mx": 32.0,
            "my": -12.0,
            "mz": 48.0,
            "temp": 26.5,
            "pressure": 1012.5,
        }

    def calibrate(self) -> bool:
        """PL: Zapisuje informację o mockowej kalibracji i zwraca sukces.

        EN: Logs the mock calibration and returns success.
        """
        logger.info("SEN0140 (mock): Calibration successful.")
        return True
