"""PL: Prosty wrapper magistrali I2C dla biblioteki smbus2.

EN: A simple I2C bus wrapper for the smbus2 library.
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
        """PL: Zastępcza klasa SMBus dla platform bez obsługi smbus2.

        EN: Fallback SMBus class for platforms without smbus2 support.
        """

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            """PL: Inicjalizuje atrapę magistrali i zgłasza tryb emulacji.

            EN: Initializes the bus stub and reports emulation mode.
            """
            logging.getLogger("I2CWrapper").warning(
                "SMBus is not available on this platform. Using Emulation Mode."
            )

        def read_byte_data(self, *args: Any, **kwargs: Any) -> int:
            """PL: Zwraca zero jako atrapę odczytu bajtu.

            EN: Returns zero as a stub byte read.
            """
            return 0

        def write_byte_data(self, *args: Any, **kwargs: Any) -> None:
            """PL: Przyjmuje zapis bajtu bez operacji sprzętowej.

            EN: Accepts a byte write without hardware access.
            """

        def read_word_data(self, *args: Any, **kwargs: Any) -> int:
            """PL: Zwraca zero jako atrapę odczytu słowa.

            EN: Returns zero as a stub word read.
            """
            return 0

        def read_i2c_block_data(self, *args: Any, **kwargs: Any) -> list[int]:
            """PL: Zwraca 32 zera jako atrapę odczytu bloku.

            EN: Returns 32 zeros as a stub block read.
            """
            return [0] * 32

        def write_i2c_block_data(self, *args: Any, **kwargs: Any) -> None:
            """PL: Przyjmuje zapis bloku bez operacji sprzętowej.

            EN: Accepts a block write without hardware access.
            """

        def close(self, *args: Any, **kwargs: Any) -> None:
            """PL: Udostępnia zgodną metodę close bez zasobu do zamknięcia.

            EN: Provides a compatible close method when there is no resource.
            """


from typing import Any


class I2CWrapper:
    """PL: Serializuje dostęp do magistrali I2C i ponawia nieudane operacje.

    EN: Serializes I2C bus access and retries failed operations.
    """

    def __init__(self, bus_num: int = 1) -> None:
        """PL: Otwiera magistralę lub przełącza wrapper w tryb emulacji.

        Args:
            bus_num: Numer magistrali I2C, zwykle 1 na Raspberry Pi.

        EN: Opens the I2C bus or switches the wrapper to emulation mode.

        Args:
            bus_num: I2C bus number, usually 1 on Raspberry Pi.
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
        self._last_reconnect_time = 0.0
        self.reconnect_on_fail = True
        if self.emulation_mode:
            self.logger.info(
                f"I2CWrapper initialized in EMULATION MODE (Bus {bus_num})"
            )
        else:
            self.logger.info(f"I2CWrapper initialized on Bus {bus_num}")

    def reconnect(self) -> bool:
        """PL: Próbuje ponownie otworzyć magistralę, ograniczając częstotliwość prób.

        EN: Attempts to reopen the bus while rate-limiting reconnection attempts.
        """
        if self.emulation_mode:
            return True

        with self._lock:
            now = time.time()
            if now - self._last_reconnect_time < 2.0:
                return False
            self._last_reconnect_time = now
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
        """PL: Odczytuje bajt z rejestru, ponawiając operację po błędzie I/O.

        EN: Reads a register byte, retrying the operation after I/O errors.
        """
        for i in range(retries):
            try:
                with self._lock:
                    return self.bus.read_byte_data(address, register)
            except OSError:
                if i == retries - 1:
                    if getattr(self, "reconnect_on_fail", True):
                        self.reconnect()
                    raise
                time.sleep(0.005)
        return 0

    def write_byte_data(
        self, address: int, register: int, value: int, retries: int = 3
    ) -> None:
        """PL: Zapisuje bajt do rejestru, ponawiając operację po błędzie I/O.

        EN: Writes a register byte, retrying the operation after I/O errors.
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
                    if getattr(self, "reconnect_on_fail", True):
                        self.reconnect()
                    raise
                time.sleep(0.005)

    def read_word_data(self, address: int, register: int, retries: int = 3) -> int:
        """PL: Odczytuje słowo z rejestru z ponowieniami po błędach I/O.

        EN: Reads a register word with retries after I/O errors.
        """
        for i in range(retries):
            try:
                with self._lock:
                    return self.bus.read_word_data(address, register)
            except OSError:
                if i == retries - 1:
                    if getattr(self, "reconnect_on_fail", True):
                        self.reconnect()
                    raise
                time.sleep(0.005)
        return 0

    def read_i2c_block_data(
        self, address: int, register: int, length: int, retries: int = 3
    ) -> list[int]:
        """PL: Odczytuje blok bajtów z rejestru z ponowieniami po błędach I/O.

        EN: Reads a register block with retries after I/O errors.
        """
        for i in range(retries):
            try:
                with self._lock:
                    return self.bus.read_i2c_block_data(address, register, length)
            except OSError:
                if i == retries - 1:
                    if getattr(self, "reconnect_on_fail", True):
                        self.reconnect()
                    raise
                time.sleep(0.005)
        return []

    def write_i2c_block_data(
        self, address: int, register: int, data: list[int], retries: int = 3
    ) -> None:
        """PL: Zapisuje blok bajtów do rejestru z ponowieniami po błędach I/O.

        EN: Writes a register block with retries after I/O errors.
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
                    if getattr(self, "reconnect_on_fail", True):
                        self.reconnect()
                    raise
                time.sleep(0.005)

    def close(self) -> None:
        """PL: Zamyka połączenie z magistralą I2C.

        EN: Closes the I2C bus connection.
        """
        with self._lock:
            try:
                self.bus.close()
            except Exception:
                pass
