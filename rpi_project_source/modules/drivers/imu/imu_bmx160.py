"""
Mockowy sterownik dla sensora BMX160.
Mock driver for the BMX160 sensor.

Ten moduł dostarcza symulowane dane dla sensora BMX160, używane do testów
systemu bez fizycznego podłączenia sprzętu.
This module provides simulated data for the BMX160 sensor, used for testing
the system without physical hardware connected.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class BMX160:
    """
    Mockowy sterownik dla sensora BMX160.
    Mock driver for the BMX160 sensor.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """
        Inicjalizuje mockowy sterownik BMX160.
        Initializes the BMX160 mock driver.

        Args:
            i2c_bus (Any): Magistrala I2C (mockowana lub rzeczywista). / I2C bus.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik BMX160 (mock) zainicjalizowany. / BMX160 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """
        Zwraca słownik z symulowanymi danymi sensora w ustandaryzowanym formacie.
        Returns a dictionary with simulated sensor data in a standardized format.

        Returns:
            dict[str, Any]: Słownik zawierający 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 'temp'.
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
        """
        Mockowa kalibracja.
        Mock calibration.
        """
        logger.info("BMX160 (mock): Calibration successful.")
        return True
