#!/usr/bin/env python3
"""PL: Odczytuje napięcie, prąd i moc zasilania przez INA219 na Raspberry Pi.

Moduł jest przeznaczony dla Waveshare UPS HAT (B) i podobnych konstrukcji.

EN: Reads supply voltage, current, and power through an INA219 on Raspberry Pi.

The module is intended for the Waveshare UPS HAT (B) and similar designs.
"""

import logging
from typing import Any

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
    )

try:
    from .native_i2c import I2CWrapper
except ImportError:
    I2CWrapper = Any  # type: ignore


class UPS:
    """PL: Monitoruje parametry zasilania mierzone przez układ INA219.

    EN: Monitors power measurements provided by the INA219 chip.
    """

    _REG_CONFIG = 0x00
    _REG_SHUNTVOLTAGE = 0x01
    _REG_BUSVOLTAGE = 0x02
    _REG_POWER = 0x03
    _REG_CURRENT = 0x04
    _REG_CALIBRATION = 0x05

    _V_FULL = 8.2  # Napięcie w pełni naładowanej baterii 2S LiPo / Voltage of a fully charged 2S LiPo battery
    _V_EMPTY = 6.0  # Napięcie rozładowanej baterii 2S LiPo / Voltage of a discharged 2S LiPo battery

    def __init__(
        self,
        logger: logging.Logger,
        i2c_wrapper: I2CWrapper,
        address: int = 0x42,
        shunt_ohms: float = 0.1,
        max_expected_amps: float = 2.0,
    ) -> None:
        """PL: Konfiguruje wrapper I2C i kalibruje INA219.

        Args:
            logger: Logger do rejestrowania zdarzeń.
            i2c_wrapper: Współdzielony wrapper magistrali I2C.
            address: Adres I2C układu INA219.
            shunt_ohms: Rezystancja bocznika w omach.
            max_expected_amps: Maksymalny spodziewany prąd w amperach.

        EN: Configures the I2C wrapper and calibrates the INA219.

        Args:
            logger: Logger for recording events.
            i2c_wrapper: Shared I2C bus wrapper.
            address: INA219 I2C address.
            shunt_ohms: Shunt resistance in ohms.
            max_expected_amps: Maximum expected current in amperes.
        """
        self.logger: logging.Logger = logger
        self.address: int = address
        self.shunt_ohms: float = shunt_ohms
        self.max_expected_amps: float = max_expected_amps
        self.i2c: I2CWrapper = i2c_wrapper
        self._current_lsb: float = 0.0
        self._power_lsb: float = 0.0
        self._notified_missing: bool = False

        if not self.i2c:
            self.logger.error("Brak wrappera I2C. UPS nie zostanie zainicjalizowany.")
            return

        try:
            self._calibrate()
            self.logger.info(
                f"INA219 pomyślnie zainicjalizowany na adresie 0x{self.address:02X}."
            )
        except Exception as e:
            self.logger.error(f"Błąd inicjalizacji INA219: {e}")
            self.i2c = None

    def _write_register(self, register: int, value: int) -> None:
        """PL: Zapisuje 16-bitową wartość do rejestru INA219.

        EN: Writes a 16-bit value to an INA219 register.
        """
        data = [(value >> 8) & 0xFF, value & 0xFF]
        self.i2c.write_i2c_block_data(self.address, register, data)

    def _read_register(self, register: int) -> int:
        """PL: Odczytuje 16-bitową wartość z rejestru INA219.

        EN: Reads a 16-bit value from an INA219 register.
        """
        data = self.i2c.read_i2c_block_data(self.address, register, 2)
        return (data[0] << 8) | data[1]

    def _calibrate(self) -> None:
        """PL: Ustawia rejestry kalibracji i konfiguracji INA219.

        EN: Sets the INA219 calibration and configuration registers.
        """
        # Określenie najmniej znaczącego bitu (LSB) dla prądu.
        # Determine the Current LSB (Least Significant Bit).
        self._current_lsb = self.max_expected_amps / 32768.0

        # Obliczenie wartości kalibracyjnej.
        # Calculate the calibration value.
        cal_value = int(0.04096 / (self._current_lsb * self.shunt_ohms))
        self._write_register(self._REG_CALIBRATION, cal_value)

        # Obliczenie LSB dla mocy.
        # Calculate the Power LSB.
        self._power_lsb = self._current_lsb * 20.0

        # Konfiguracja rejestru. Używane są domyślne ustawienia z biblioteki Adafruit,
        # ale z modyfikacjami dla zakresu 16V i wzmocnienia /4 (160mV).
        # Configure the register. Using default settings from Adafruit's library
        # but modified for 16V range and /4 gain (160mV).

        # Reset
        config = 0b0000_0001_1001_1111

        # Ustawienie zakresu napięcia na 16V (bit 13 = 0).
        # Set Bus Voltage Range to 16V (bit 13 = 0).
        config &= ~0x2000

        # Ustawienie wzmocnienia na /4 (160mV) (bity 11,12 = 10).
        # Set Gain to /4 (160mV) (bits 11,12 = 10).
        config &= ~0x1800  # Najpierw wyczyść bity wzmocnienia / First, clear gain bits
        config |= 0b10 << 11

        # Ustawienie rozdzielczości ADC dla napięcia na 12 bitów, 8 próbek (3.4ms).
        # Set Bus ADC Resolution to 12-bit, 8 samples (3.4ms).
        config |= 0b0110 << 7

        # Ustawienie rozdzielczości ADC dla bocznika na 12 bitów, 8 próbek (3.4ms).
        # Set Shunt ADC Resolution to 12-bit, 8 samples (3.4ms).
        config |= 0b0110 << 3

        # Ustawienie trybu pracy na ciągły pomiar napięcia i prądu.
        # Set Mode to Shunt and Bus, Continuous.
        config |= 0b111

        self._write_register(self._REG_CONFIG, config)
        self.logger.debug(
            f"INA219 skalibrowany. Wartość kalibracji: {cal_value}, config: {bin(config)}"
        )

    def read_data(self) -> dict[str, float]:
        """PL: Odczytuje napięcie, prąd, moc i szacowany procent baterii.

        Returns:
            Słownik: voltage w V, current w mA, power w W i percent w %.

        EN: Reads voltage, current, power, and estimated battery percentage.

        Returns:
            Dictionary with voltage in V, current in mA, power in W, and percent.
        """
        data = {"voltage": 0.0, "current": 0.0, "power": 0.0, "percent": 0.0}
        if not self.i2c:
            if not self._notified_missing:
                self.logger.warning(
                    "Nie można odczytać danych, INA219 nie jest zainicjalizowany."
                )
                self._notified_missing = True
            return data

        try:
            raw_voltage = self._read_register(self._REG_BUSVOLTAGE)
            data["voltage"] = (raw_voltage >> 3) * 0.004
            raw_current = self._read_register(self._REG_CURRENT)
            if raw_current > 32767:
                raw_current -= 65536
            data["current"] = raw_current * self._current_lsb * 1000
            raw_power = self._read_register(self._REG_POWER)
            data["power"] = raw_power * self._power_lsb
            percent = (
                (data["voltage"] - self._V_EMPTY) / (self._V_FULL - self._V_EMPTY)
            ) * 100
            data["percent"] = max(0.0, min(100.0, round(percent, 1)))
        except Exception as e:
            self.logger.error(f"Błąd podczas odczytu danych z INA219: {e}")

        return data

    def cleanup(self) -> None:
        """PL: Udostępnia metodę cleanup; magistrala I2C jest współdzielona.

        EN: Provides a cleanup method; the I2C bus is shared.
        """


if __name__ == "__main__":
    import time

    from native_i2c import I2CWrapper

    logging.info("Uruchomiono test modułu UPS (INA219)...")
    test_logger = logging.getLogger("UPS_Test")

    try:
        i2c = I2CWrapper(1)
        ups_sensor = UPS(logger=test_logger, i2c_wrapper=i2c)

        if ups_sensor.i2c is None:
            logging.error("Nie udało się zainicjalizować czujnika.")
        else:
            try:
                while True:
                    telemetry = ups_sensor.read_data()
                    logging.info("-" * 30)
                    logging.info(f"Napięcie: {telemetry['voltage']:.2f} V")
                    logging.info(f"Prąd:     {telemetry['current']:.2f} mA")
                    logging.info(f"Moc:      {telemetry['power']:.2f} W")
                    logging.info(f"Bateria:  {telemetry['percent']:.1f} %")
                    time.sleep(2)
            except KeyboardInterrupt:
                logging.info("\nZakończono test.")
            finally:
                ups_sensor.cleanup()
                i2c.close()

    except Exception as e:
        logging.error(f"I2C Init failed: {e}")
