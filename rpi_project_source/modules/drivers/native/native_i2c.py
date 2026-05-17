"""
A simple I2C wrapper for smbus2.
Prosty wrapper I2C dla smbus2.
"""

import logging
import threading
import time
from typing import Any

# On non-Linux platforms smbus2 (and fcntl) is not available. Provide
# a safe fallback so tests can import this module on Windows.
try:
    from smbus2 import SMBus
except Exception:  # pragma: no cover - platform-specific fallback

    class SMBus:  # type: ignore
        """
        Zastępcza klasa SMBus dla platform Windows/Mac.
        Mock SMBus class for non-Linux platforms.
        """

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            """Inicjalizacja (MOCK). / Init (MOCK)."""
            logging.getLogger("I2CWrapper").warning(
                "SMBus is not available on this platform. Using Emulation Mode."
            )

        def read_byte_data(self, *args: Any, **kwargs: Any) -> int:
            """Odczyt bajtu (MOCK). / Read byte (MOCK)."""
            return 0

        def write_byte_data(self, *args: Any, **kwargs: Any) -> None:
            """Zapis bajtu (MOCK). / Write byte (MOCK)."""

        def read_word_data(self, *args: Any, **kwargs: Any) -> int:
            """Odczyt słowa (MOCK). / Read word (MOCK)."""
            return 0

        def read_i2c_block_data(self, *args: Any, **kwargs: Any) -> list[int]:
            """Odczyt bloku (MOCK). / Read block (MOCK)."""
            return [0] * 32

        def write_i2c_block_data(self, *args: Any, **kwargs: Any) -> None:
            """Zapis bloku (MOCK). / Write block (MOCK)."""

        def close(self, *args: Any, **kwargs: Any) -> None:
            """Zamyka połączenie (MOCK). / Closes connection (MOCK)."""


from typing import Any


class I2CWrapper:
    """
    A wrapper for the smbus2.SMBus class to handle I2C communication.
    Wrapper dla klasy smbus2.SMBus do obsługi komunikacji I2C.
    """

    def __init__(self, bus_num: int = 1) -> None:
        """
        Initializes the I2C bus.
        Inicjalizuje magistralę I2C.

        Args:
            bus_num (int): The I2C bus number (e.g., 1 for Raspberry Pi).
                           Numer magistrali I2C (np. 1 dla Raspberry Pi).
        """
        self.bus_num = bus_num
        self.logger = logging.getLogger("I2CWrapper")
        try:
            self.bus = SMBus(bus_num)
            self.emulation_mode = (
                not hasattr(self.bus, "open") and SMBus.__module__ != "smbus2"
            )
        except Exception as e:
            self.logger.warning(
                f"Failed to initialize SMBus {bus_num}: {e}. Falling back to emulation."
            )
            self.bus = SMBus()  # Use stub
            self.emulation_mode = True

        self._lock = threading.Lock()
        if self.emulation_mode:
            self.logger.info(
                f"I2CWrapper initialized in EMULATION MODE (Bus {bus_num})"
            )
        else:
            self.logger.info(f"I2CWrapper initialized on Bus {bus_num}")

    def reconnect(self) -> bool:
        """
        Próbuje zamknąć i otworzyć ponownie magistralę I2C. [PLAN-011]
        Attempts to close and reopen the I2C bus.
        """
        if self.emulation_mode:
            return True

        with self._lock:
            try:
                self.logger.warning(
                    f"I2C: HARD RECONNECT triggered on Bus {self.bus_num}..."
                )
                try:
                    self.bus.close()
                except Exception:
                    pass
                time.sleep(0.2)  # Longer wait for bus stabilization
                self.bus = SMBus(self.bus_num)
                self.logger.info(f"I2C: Bus {self.bus_num} reopened successfully.")
                return True
            except Exception as e:
                self.logger.error(f"I2C: CRITICAL BUS RECOVERY FAILED: {e}")
                return False

    def read_byte_data(self, address: int, register: int, retries: int = 3) -> int:
        """
        Odczytuje pojedynczy bajt z zadanego rejestru I2C z ponowieniami.
        Reads a single byte from a given register of an I2C device with retries.
        """
        for i in range(retries):
            try:
                with self._lock:
                    return self.bus.read_byte_data(address, register)
            except OSError:
                if i == retries - 1:
                    self.reconnect()
                    raise
                time.sleep(0.005)
        return 0

    def write_byte_data(
        self, address: int, register: int, value: int, retries: int = 3
    ) -> None:
        """
        Zapisuje pojedynczy bajt do danego rejestru urządzenia I2C z ponowieniami.
        """
        for i in range(retries):
            try:
                with self._lock:
                    self.bus.write_byte_data(address, register, value)
                return
            except OSError as e:
                self.logger.warning(
                    f"I2C Write Error (Addr 0x{address:02x}, Reg 0x{register:02x}): {e}. "
                    f"Retry {i+1}/{retries}"
                )
                if i == retries - 1:
                    self.reconnect()
                    raise
                time.sleep(0.005)

    def read_word_data(self, address: int, register: int, retries: int = 3) -> int:
        """
        Odczytuje dwubajtowe słowo z rejestru urządzenia z ponowieniami.
        """
        for i in range(retries):
            try:
                with self._lock:
                    return self.bus.read_word_data(address, register)
            except OSError:
                if i == retries - 1:
                    self.reconnect()
                    raise
                time.sleep(0.005)
        return 0

    def read_i2c_block_data(
        self, address: int, register: int, length: int, retries: int = 3
    ) -> list[int]:
        """
        Odczytuje blok danych z danego rejestru z ponowieniami.
        """
        for i in range(retries):
            try:
                with self._lock:
                    return self.bus.read_i2c_block_data(address, register, length)
            except OSError:
                if i == retries - 1:
                    self.reconnect()
                    raise
                time.sleep(0.005)
        return []

    def write_i2c_block_data(
        self, address: int, register: int, data: list[int], retries: int = 3
    ) -> None:
        """
        Zapisuje blok danych do danego rejestru z ponowieniami.
        """
        for i in range(retries):
            try:
                with self._lock:
                    self.bus.write_i2c_block_data(address, register, data)
                return
            except OSError as e:
                self.logger.warning(
                    f"I2C Block Write Error (Addr 0x{address:02x}, Reg 0x{register:02x}): {e}. "
                    f"Retry {i+1}/{retries}"
                )
                if i == retries - 1:
                    self.reconnect()
                    raise
                time.sleep(0.005)

    def close(self) -> None:
        """
        Closes the I2C bus connection.
        """
        with self._lock:
            try:
                self.bus.close()
            except Exception:
                pass
