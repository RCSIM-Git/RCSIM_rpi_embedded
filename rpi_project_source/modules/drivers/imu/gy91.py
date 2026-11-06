"""PL: Udostępnia mockowy sterownik GY-91 z przykładowymi danymi 10-DoF.

EN: Provides a mock GY-91 driver with sample 10-DoF data.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class GY91:
    """PL: Zwraca wyłącznie symulowane dane modułu GY-91.

    EN: Returns simulated data for the GY-91 module only.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """PL: Zapisuje magistralę I2C przekazaną do mockowego sterownika.

        EN: Stores the I2C bus passed to the mock driver.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik GY91 (mock) zainicjalizowany. / GY91 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """PL: Zwraca symulowane dane IMU i barometru w ustandaryzowanym słowniku.

        EN: Returns simulated IMU and barometer data in a standardized dictionary.
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
            "pressure": 1013.25,
        }

    def calibrate(self) -> bool:
        """PL: Zapisuje informację o mockowej kalibracji i zwraca sukces.

        EN: Logs the mock calibration and returns success.
        """
        logger.info("GY91 (mock): Calibration successful.")
        return True
