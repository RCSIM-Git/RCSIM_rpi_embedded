"""
Mockowy sterownik dla sensora ciśnienia/temperatury BMP388.
Mock driver for the BMP388 pressure/temperature sensor.

Ten moduł dostarcza symulowane dane dla sensora BMP388, używane do testów
altimetru i termometru.
This module provides simulated data for the BMP388 sensor, used for testing
the altimeter and thermometer.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class BMP388:
    """
    Mockowy sterownik dla sensora BMP388.
    Mock driver for the BMP388 sensor.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """
        Inicjalizuje mockowy sterownik BMP388.
        Initializes the BMP388 mock driver.

        Args:
            i2c_bus (Any): Magistrala I2C. / I2C bus.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik BMP388 (mock) zainicjalizowany. / BMP388 mock driver initialized."
        )

    def read_data(self) -> dict[str, float]:
        """
        Zwraca słownik z symulowanymi danymi ciśnienia i temperatury.
        Returns a dictionary with simulated pressure and temperature data.

        Returns:
            dict[str, float]: Słownik zawierający temperaturę, ciśnienie i wysokość.
                              / Dictionary containing temperature, pressure, and altitude.
        """
        return {"temperature": 25.5, "pressure": 1013.5, "altitude": 120.5}

    def calibrate(self) -> bool:
        """
        Mockowa kalibracja.
        Mock calibration.
        """
        logger.info("BMP388 (mock): Calibration successful.")
        return True
