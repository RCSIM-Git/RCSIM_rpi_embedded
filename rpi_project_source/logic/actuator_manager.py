"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Udostępnia interfejs do sterowania dodatkowymi serwami przez HardwareManager.
EN: Provides an interface for controlling additional servos through HardwareManager.
"""

import logging
from typing import TYPE_CHECKING

# Uniknięcie pętli importów - HardwareManager importuje ten moduł
# Avoid import loops - HardwareManager imports this module
if TYPE_CHECKING:
    from modules.managers.hardware_manager import HardwareManager


logger = logging.getLogger(__name__)


class ActuatorManager:
    """PL: Zarządza dodatkowymi serwami i aktywatorami, takimi jak chwytak.
    EN: Manages additional servos and actuators, such as a gripper.
    """

    def __init__(self, hw_manager: "HardwareManager"):
        """PL: Zapisuje referencję do głównego menedżera sprzętu.
        EN: Stores a reference to the main hardware manager.
        """
        self.hw_manager = hw_manager
        logger.info("ActuatorManager zainicjalizowany.")

    def set_gripper(self, value: float) -> None:
        """PL: Ustawia pozycję chwytaka na podstawie wartości od -1 do 1.
        EN: Sets the gripper position from a value in the range -1 to 1.
        """
        # Logika specyficzna dla tego menedżera
        # Logic specific to this manager
        logger.info(f"Ustawiono pozycję chwytaka na: {value}")

        # Wywołanie niskopoziomowej metody w HardwareManager
        # Call low-level method in HardwareManager
        # Zakładamy, że HardwareManager ma metodę `set_servo_pulse` przyjmującą numer kanału i wartość w mikrosekundach.
        # We assume HardwareManager has a `set_servo_pulse` method accepting channel number and value in microseconds.
        # Konwertujemy wartość -1..1 na puls 1000..2000 us.
        # Convert -1..1 value to 1000..2000 us pulse.
        pulse_us = int(1500 + (value * 500))

        # Zakładamy, że chwytak jest podłączony do kanału PWM nr 2
        # We assume the gripper is connected to PWM channel #2
        gripper_channel = 2

        if hasattr(self.hw_manager, "set_servo_pulse"):
            self.hw_manager.set_servo_pulse(gripper_channel, pulse_us)
        else:
            logger.warning(
                "Metoda 'set_servo_pulse' nie jest dostępna w HardwareManager."
            )

    def cleanup(self) -> None:
        """PL: Wykonuje zamknięcie menedżera aktywatorów.
        EN: Performs actuator manager shutdown.
        """
        logger.info("ActuatorManager wykonuje cleanup.")
        # W tym prostym przypadku nie ma zasobów do zwolnienia
        # In this simple case, there are no resources to free
