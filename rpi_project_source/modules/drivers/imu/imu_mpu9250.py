"""
Mockowy sterownik dla sensora MPU9250.
Mock driver for the MPU9250 sensor.

Ten moduł dostarcza symulowane dane dla sensora MPU9250, używane do testów
systemu bez fizycznego podłączenia sprzętu.
This module provides simulated data for the MPU9250 sensor, used for testing
the system without physical hardware connected.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class MPU9250:
    """
    Mockowy sterownik dla sensora MPU9250.
    Mock driver for the MPU9250 sensor.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """
        Inicjalizuje mockowy sterownik MPU9250.
        Initializes the MPU9250 mock driver.

        Args:
            i2c_bus (Any): Magistrala I2C (mockowana lub rzeczywista). / I2C bus.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik MPU9250 (mock) zainicjalizowany. / MPU9250 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """
        Zwraca słownik z symulowanymi danymi sensora w ustandaryzowanym formacie.
        Returns a dictionary with simulated sensor data in a standardized format.

        Returns:
            dict[str, Any]: Słownik zawierający 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 'temp'.
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
        """
        Mockowa kalibracja.
        Mock calibration.
        """
        logger.info("MPU9250 (mock): Calibration successful.")
        return True
