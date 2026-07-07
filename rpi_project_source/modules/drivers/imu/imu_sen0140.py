"""
Mockowy sterownik dla sensora DFRobot SEN0140 (10 DOF IMU).
Mock driver for the DFRobot SEN0140 sensor (10 DOF IMU).

Ten moduł dostarcza symulowane dane dla sensora SEN0140 (ADXL345 + ITG3205 + VCM5883L + BMP280), 
używane do testów systemu bez fizycznego podłączenia sprzętu.
This module provides simulated data for the SEN0140 sensor (ADXL345 + ITG3205 + VCM5883L + BMP280), 
used for testing the system without physical hardware connected.
"""

import logging
from typing import Any

logger: logging.Logger = logging.getLogger(__name__)


class SEN0140:
    """
    Sterownik dla sensora SEN0140 (ADXL345 + ITG3205 + VCM5883L + BMP280).
    Driver for the SEN0140 sensor (ADXL345 + ITG3205 + VCM5883L + BMP280).

    Na razie zwraca tylko mockowe dane.
    Currently returns mock data only.
    """

    def __init__(self, i2c_bus: Any) -> None:
        """
        Inicjalizuje mockowy sterownik SEN0140.
        Initializes the SEN0140 mock driver.

        Args:
            i2c_bus (Any): Magistrala I2C (mockowana lub rzeczywista). / I2C bus.
        """
        self.i2c: Any = i2c_bus
        logger.info(
            "Sterownik SEN0140 (mock) zainicjalizowany. / SEN0140 mock driver initialized."
        )

    def read_data(self) -> dict[str, Any]:
        """
        Zwraca słownik z symulowanymi danymi sensora 10-DoF w ustandaryzowanym formacie.
        Returns a dictionary with simulated 10-DoF sensor data in a standardized format.

        Returns:
            dict[str, Any]: Słownik zawierający 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 'temp', 'pressure'.
        """
        return {
            "ax": 9.81,
            "ay": 0.05,
            "az": 0.15,
            "gx": 0.015,
            "gy": 0.025,
            "gz": 0.035,
            "mx": 32.0,
            "my": -12.0,
            "mz": 48.0,
            "temp": 26.5,
            "pressure": 1012.5,
        }

    def calibrate(self) -> bool:
        """
        Mockowa kalibracja.
        Mock calibration.
        """
        logger.info("SEN0140 (mock): Calibration successful.")
        return True
