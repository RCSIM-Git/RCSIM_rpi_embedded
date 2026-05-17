"""
Przykładowy sterownik dla sensora GY-91 (MPU-9250 + BMP280).
Example driver for the GY-91 sensor (MPU-9250 + BMP280).

Ten moduł dostarcza sterownik do obsługi popularnego modułu GY-91.
This module provides a driver to handle the popular GY-91 module.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class GY91:
    """
    Sterownik dla sensora GY-91 (MPU-9250 + BMP280).
    Driver for the GY-91 sensor (MPU-9250 + BMP280).

    Na razie zwraca tylko mockowe dane.
    Currently returns mock data only.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """
        Inicjalizuje sterownik GY91.
        Initializes the GY91 driver.

        Args:
            i2c_bus (Any): Magistrala I2C. / I2C bus.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik GY91 (mock) zainicjalizowany. / GY91 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """
        Zwraca słownik z danymi sensora w ustandaryzowanym formacie.
        Returns a dictionary with sensor data in a standardized format.

        Returns:
            dict[str, Any]: Słownik z danymi 10-DoF (ax, ay, az, gx, gy, gz, mx, my, mz, temp, pressure).
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
        """
        Mockowa kalibracja.
        Mock calibration.
        """
        logger.info("GY91 (mock): Calibration successful.")
        return True
