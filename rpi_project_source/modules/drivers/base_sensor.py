"""
Moduł klas bazowych dla sensorów sprzętowych.
Base classes module for hardware sensors.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from modules.drivers.native.native_i2c import I2CWrapper

logger = logging.getLogger(__name__)


class IMUBase(ABC):
    """
    Abstrakcyjna klasa bazowa dla czujników inercyjnych (IMU).
    Abstract base class for Inertial Measurement Unit (IMU) sensors.

    Podklasy powinny nadpisać DRIVER_NAME, I2C_ADDRESSES i PRIORITY
    aby umożliwić automatyczną detekcję przez SensorRegistry.
    Subclasses should override DRIVER_NAME, I2C_ADDRESSES and PRIORITY
    to enable automatic detection via SensorRegistry.
    """

    # --- Metadane Registry Pattern / Registry Pattern metadata ---
    DRIVER_NAME: str = ""
    """Unikalna nazwa sterownika, np. 'native_bno08x'. / Unique driver name."""

    I2C_ADDRESSES: list[int] = []
    """Lista adresów I2C sensora. / I2C addresses list."""

    PRIORITY: int = 100
    """Priorytet detekcji — niższy = wyższy priorytet. / Detection priority — lower = higher."""

    @classmethod
    def scan(cls, i2c: I2CWrapper) -> bool:
        """
        Sprawdza czy sensor jest obecny na magistrali I2C.
        Checks whether the sensor is present on the I2C bus.

        Domyślna implementacja próbuje odczytać bajt z każdego adresu.
        Podklasy mogą nadpisać tę metodę aby dokonać weryfikacji WHO_AM_I.
        Default implementation tries to read a byte from each address.
        Subclasses can override to perform WHO_AM_I verification.

        Args:
            i2c (I2CWrapper): Wrapper magistrali I2C. / I2C bus wrapper.

        Returns:
            bool: True jeśli sensor wykryty. / True if sensor detected.
        """
        for addr in cls.I2C_ADDRESSES:
            try:
                i2c.read_byte_data(addr, 0x00)
                return True
            except (OSError, IOError):
                continue
        return False

    @abstractmethod
    def read_data(self) -> dict[str, Any]:
        """
        Odczytuje dane z sensora i zwraca je w ustandaryzowanym formacie.
        Reads data from the sensor and returns it in a standardized format.

        Returns:
            dict[str, Any]: Słownik z danymi sensora. / Dictionary with sensor data.
                            Oczekiwane klucze / Expected keys:
                            'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                            'mx', 'my', 'mz', 'temp'.
        """
        raise NotImplementedError

    @abstractmethod
    def calibrate(self) -> bool:
        """
        Przeprowadza kalibrację sensora (np. wyznaczenie biasu).
        Performs sensor calibration (e.g., bias estimation).

        Returns:
            bool: True jeśli kalibracja się powiodła, False w przeciwnym razie.
                  True if calibration succeeded, False otherwise.
        """
        raise NotImplementedError
