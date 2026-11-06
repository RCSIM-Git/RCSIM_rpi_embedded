"""PL: Udostępnia mockowe odczyty ciśnienia i temperatury BMP388.

EN: Provides mock pressure and temperature readings for the BMP388.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class BMP388:
    """PL: Zwraca symulowane pomiary sensora BMP388.

    EN: Returns simulated BMP388 sensor measurements.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """PL: Zapisuje magistralę I2C przekazaną do mockowego sterownika.

        EN: Stores the I2C bus passed to the mock driver.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik BMP388 (mock) zainicjalizowany. / BMP388 mock driver initialized."
        )

    def read_data(self) -> dict[str, float]:
        """PL: Zwraca symulowaną temperaturę, ciśnienie i wysokość.

        EN: Returns simulated temperature, pressure, and altitude.
        """
        return {"temperature": 25.5, "pressure": 1013.5, "altitude": 120.5}

    def calibrate(self) -> bool:
        """PL: Zapisuje informację o mockowej kalibracji i zwraca sukces.

        EN: Logs the mock calibration and returns success.
        """
        logger.info("BMP388 (mock): Calibration successful.")
        return True
