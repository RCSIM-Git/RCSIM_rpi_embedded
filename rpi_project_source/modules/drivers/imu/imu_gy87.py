"""PL: Udostępnia mockowe dane modułu wielosensorowego GY-87.

EN: Provides mock data for the GY-87 multi-sensor module.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class GY87:
    """PL: Zwraca symulowane pomiary modułu GY-87.

    EN: Returns simulated readings from the GY-87 module.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """PL: Zapisuje magistralę I2C przekazaną do mockowego sterownika.

        EN: Stores the I2C bus passed to the mock driver.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik GY87 (mock) zainicjalizowany. / GY87 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """PL: Zwraca symulowane dane IMU w ustandaryzowanym słowniku.

        EN: Returns simulated IMU data in a standardized dictionary.
        """
        return {
            "ax": 9.7,
            "ay": 0.2,
            "az": 0.1,
            "gx": 0.02,
            "gy": 0.01,
            "gz": 0.03,
            "mx": 31.0,
            "my": -16.0,
            "mz": 51.0,
            "temp": 25.0,
        }

    def calibrate(self) -> bool:
        """PL: Zapisuje informację o mockowej kalibracji i zwraca sukces.

        EN: Logs the mock calibration and returns success.
        """
        logger.info("GY87 (mock): Calibration successful.")
        return True
