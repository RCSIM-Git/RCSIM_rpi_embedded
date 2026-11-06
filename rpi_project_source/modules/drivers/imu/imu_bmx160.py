"""PL: Udostępnia mockowe odczyty BMX160 do testów bez sprzętu.

EN: Provides mock BMX160 readings for testing without physical hardware.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class BMX160:
    """PL: Zwraca symulowane pomiary sensora BMX160.

    EN: Returns simulated BMX160 sensor readings.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """PL: Zapisuje magistralę I2C przekazaną do mockowego sterownika.

        EN: Stores the I2C bus passed to the mock driver.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik BMX160 (mock) zainicjalizowany. / BMX160 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """PL: Zwraca symulowane dane IMU w ustandaryzowanym słowniku.

        EN: Returns simulated IMU data in a standardized dictionary.
        """
        return {
            "ax": 9.8,
            "ay": 0.1,
            "az": 0.2,
            "gx": 0.01,
            "gy": 0.02,
            "gz": 0.03,
            "mx": 30.0,
            "my": -15.0,
            "mz": 50.0,
            "temp": 25.0,
        }

    def calibrate(self) -> bool:
        """PL: Zapisuje informację o mockowej kalibracji i zwraca sukces.

        EN: Logs the mock calibration and returns success.
        """
        logger.info("BMX160 (mock): Calibration successful.")
        return True
