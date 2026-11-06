"""PL: Udostępnia mockowe dane MPU9250 do testów bez sprzętu.

EN: Provides mock MPU9250 data for testing without physical hardware.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class MPU9250:
    """PL: Zwraca symulowane pomiary sensora MPU9250.

    EN: Returns simulated readings from the MPU9250 sensor.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """PL: Zapisuje magistralę I2C przekazaną do mockowego sterownika.

        EN: Stores the I2C bus passed to the mock driver.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik MPU9250 (mock) zainicjalizowany. / MPU9250 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """PL: Zwraca symulowane dane IMU w ustandaryzowanym słowniku.

        EN: Returns simulated IMU data in a standardized dictionary.
        """
        return {
            "ax": 9.6,
            "ay": 0.3,
            "az": 0.1,
            "gx": 0.03,
            "gy": 0.01,
            "gz": 0.02,
            "mx": 29.0,
            "my": -14.0,
            "mz": 49.0,
            "temp": 25.0,
        }

    def calibrate(self) -> bool:
        """PL: Zapisuje informację o mockowej kalibracji i zwraca sukces.

        EN: Logs the mock calibration and returns success.
        """
        logger.info("MPU9250 (mock): Calibration successful.")
        return True
