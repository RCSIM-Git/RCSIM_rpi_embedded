"""PL: Moduł klas bazowych dla sensorów sprzętowych.

EN: Base classes module for hardware sensors.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from modules.drivers.native.native_i2c import I2CWrapper

logger = logging.getLogger(__name__)


class IMUBase(ABC):
    """PL:
    Abstrakcyjna klasa bazowa dla czujników inercyjnych (IMU).
    EN: Abstract base class for Inertial Measurement Unit (IMU) sensors.

    Podklasy powinny nadpisać DRIVER_NAME, I2C_ADDRESSES i PRIORITY
    aby umożliwić automatyczną detekcję przez SensorRegistry.
    Subclasses should override DRIVER_NAME, I2C_ADDRESSES and PRIORITY
    to enable automatic detection via SensorRegistry.
    """

    # --- Metadane Registry Pattern / Registry Pattern metadata ---
    DRIVER_NAME: str = ""
    """PL: Unikalna nazwa sterownika. EN: Unique driver name."""

    I2C_ADDRESSES: list[int] = []
    """PL: Lista adresów I2C sensora. EN: Sensor I2C addresses."""

    PRIORITY: int = 100
    """PL: Priorytet detekcji; niższa wartość jest sprawdzana wcześniej.

    EN: Detection priority; lower values are checked first.
    """

    @classmethod
    def scan(cls, i2c: I2CWrapper) -> bool:
        """PL:
        Sprawdza czy sensor jest obecny na magistrali I2C.
        EN: Checks whether the sensor is present on the I2C bus.

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
        """PL:
        Odczytuje dane z sensora i zwraca je w ustandaryzowanym formacie.
        EN: Reads data from the sensor and returns it in a standardized format.

        Returns:
            dict[str, Any]: Słownik z danymi sensora. / Dictionary with sensor data.
                            Oczekiwane klucze / Expected keys:
                            'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                            'mx', 'my', 'mz', 'temp'.
        """
        raise NotImplementedError

    @abstractmethod
    def calibrate(self) -> bool:
        """PL:
        Przeprowadza kalibrację sensora (np. wyznaczenie biasu).
        EN: Performs sensor calibration (e.g., bias estimation).

        Returns:
            bool: True jeśli kalibracja się powiodła, False w przeciwnym razie.
                  True if calibration succeeded, False otherwise.
        """
        raise NotImplementedError
