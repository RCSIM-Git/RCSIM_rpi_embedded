# The MIT License (MIT)
# Copyright (c) 2026 RCSIM / Mateusz Buzek. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
# OR OTHER DEALINGS IN THE SOFTWARE.

"""
Native PCA9685 Driver using a shared I2C wrapper.
Natywny sterownik PCA9685 używający współdzielonego wrappera I2C.

This driver provides a low-level interface to the PCA9685 16-channel, 12-bit
PWM/Servo Driver. It is designed to be lightweight and depends only on the
`smbus2` library via the provided `I2CWrapper`.

Ten sterownik dostarcza niskopoziomowy interfejs do 16-kanałowego, 12-bitowego
sterownika PWM/Serwo PCA9685. Został zaprojektowany jako lekki i zależy tylko od
biblioteki `smbus2` poprzez dostarczony `I2CWrapper`.
"""

import time

from .native_i2c import I2CWrapper


class NativePCA9685:
    """
    A lightweight, direct-register-access driver for the PCA9685 PWM
    controller.
    Lekki sterownik z bezpośrednim dostępem do rejestrów dla kontrolera PWM
    PCA9685.
    """

    # --- Register Definitions ---
    _MODE1 = 0x00
    _MODE2 = 0x01
    _SUBADR1 = 0x02
    _SUBADR2 = 0x03
    _SUBADR3 = 0x04
    _PRE_SCALE = 0xFE
    _LED0_ON_L = 0x06
    _LED0_ON_H = 0x07
    _LED0_OFF_L = 0x08
    _LED0_OFF_H = 0x09
    _ALL_LED_ON_L = 0xFA
    _ALL_LED_ON_H = 0xFB
    _ALL_LED_OFF_L = 0xFC
    _ALL_LED_OFF_H = 0xFD

    # --- Mode Bits ---
    _RESTART = 0x80
    _SLEEP = 0x10
    _ALLCALL = 0x01
    _INVRT = 0x10
    _OUTDRV = 0x04
    _AUTOINC = 0x20

    def __init__(
        self,
        i2c_wrapper: I2CWrapper,
        address: int = 0x40,
        reference_clock_speed: int = 25000000,
    ) -> None:
        """
        Initializes the PCA9685 driver.
        Inicjalizuje sterownik PCA9685.
        """
        self.i2c: I2CWrapper = i2c_wrapper
        self.address: int = address
        self.reference_clock_speed: int = reference_clock_speed
        self._frequency: int = 50  # Store current frequency
        self.reset()
        self.set_frequency(50)  # Default to 50Hz, common for servos

    def reset(self) -> None:
        """
        Resets the PCA9685 device to its default state.
        Resetuje urządzenie PCA9685 do stanu domyślnego.
        """
        # Set MODE1: disable sleep, enable AUTOINC [PLAN-003]
        self.i2c.write_byte_data(self.address, self._MODE1, self._AUTOINC)
        time.sleep(0.01)  # Wait for oscillator to stabilize

        # Configure MODE2 for totem-pole outputs (required for servos)
        self.i2c.write_byte_data(self.address, self._MODE2, self._OUTDRV)
        time.sleep(0.01)

    def set_frequency(self, freq: int) -> None:
        """
        Sets the PWM frequency for all channels.

        Args:
            freq (int): The desired frequency in Hz.
        """
        self._frequency = freq  # Update stored frequency

        prescaleval = float(self.reference_clock_speed)
        prescaleval /= 4096.0  # 12-bit resolution
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)

        old_mode = self.i2c.read_byte_data(self.address, self._MODE1)
        new_mode = (old_mode & ~self._RESTART) | self._SLEEP  # Set sleep bit
        self.i2c.write_byte_data(self.address, self._MODE1, new_mode)
        self.i2c.write_byte_data(self.address, self._PRE_SCALE, prescale)
        self.i2c.write_byte_data(self.address, self._MODE1, old_mode)
        time.sleep(0.005)  # Wait for oscillator to stabilize
        self.i2c.write_byte_data(self.address, self._MODE1, old_mode | self._AUTOINC)

    def set_all_pwm(self, on: int, off: int) -> None:
        """Sets all PWM channels."""
        self.i2c.write_byte_data(self.address, self._ALL_LED_ON_L, on & 0xFF)
        self.i2c.write_byte_data(self.address, self._ALL_LED_ON_H, on >> 8)
        self.i2c.write_byte_data(self.address, self._ALL_LED_OFF_L, off & 0xFF)
        self.i2c.write_byte_data(self.address, self._ALL_LED_OFF_H, off >> 8)

    def set_pwm(self, channel: int, on_tick: int, off_tick: int) -> None:
        """
        Sets a single PWM channel's on and off ticks.
        Ustawia tiki "on" i "of" dla pojedynczego kanału PWM.

        Args:
            channel (int): The channel to set (0-15).
            on_tick (int): The tick (0-4095) when the signal should go high.
            off_tick (int): The tick (0-4095) when the signal should go low.
        """
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")

        base_reg = self._LED0_ON_L + 4 * channel

        # [PLAN-003] Atomowy zapis 4 bajtów (ON_L, ON_H, OFF_L, OFF_H)
        # Wymaga włączonego _AUTOINC w rejestrze MODE1.
        data = [on_tick & 0xFF, on_tick >> 8, off_tick & 0xFF, off_tick >> 8]
        self.i2c.write_i2c_block_data(self.address, base_reg, data)

    def set_us(self, channel: int, microseconds: int) -> None:
        """
        Sets the PWM pulse width for a channel in microseconds.
        Ustawia szerokość impulsu PWM dla kanału w mikrosekundach.
        """
        # Calculate the duration of a single tick in microseconds
        tick_duration_us = (1000000.0 / self._frequency) / 4096.0

        # Convert the desired microsecond pulse width into a number of ticks
        off_tick = int(microseconds / tick_duration_us)

        # Typically, the pulse starts at tick 0 and ends at the calculated off_tick
        self.set_pwm(channel, 0, off_tick)
