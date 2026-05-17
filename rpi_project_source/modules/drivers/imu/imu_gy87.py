"""
Mockowy sterownik dla wielosensorowego modułu GY-87.
Mock driver for the GY-87 multi-sensor module.

Ten moduł dostarcza symulowane dane dla modułu GY-87 (MPU6050 + HMC5883L + BMP180).
This module provides simulated data for the GY-87 module (MPU6050 + HMC5883L + BMP180).
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class GY87:
    """
    Mockowy sterownik dla sensora GY-87.
    Mock driver for the GY-87 sensor.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """
        Inicjalizuje mockowy sterownik GY87.
        Initializes the GY87 mock driver.

        Args:
            i2c_bus (Any): Magistrala I2C. / I2C bus.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik GY87 (mock) zainicjalizowany. / GY87 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """
        Zwraca słownik z symulowanymi danymi sensora 10-DoF w ustandaryzowanym formacie.
        Returns a dictionary with simulated 10-DoF sensor data in a standardized format.

        Returns:
            dict[str, Any]: Słownik zawierający 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 'temp'.
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
        """
        Mockowa kalibracja.
        Mock calibration.
        """
        logger.info("GY87 (mock): Calibration successful.")
        return True
