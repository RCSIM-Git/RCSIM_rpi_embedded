"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Moduł Zarządcy Aktywatorów (Actuator Manager) dla RPi.
Actuator Manager Module for RPi.

Ten moduł jest odpowiedzialny za sterowanie dodatkowymi aktywatorami
i serwomechanizmami, które nie są bezpośrednio związane z napędem
pojazdu, takimi jak chwytaki, ramiona robotyczne, itp.
This module is responsible for controlling additional actuators
and servos that are not directly related to the vehicle drivetrain,
such as grippers, robotic arms, etc.

Kluczowe funkcjonalności / Key features:
-   **Abstrakcja sterowania / Control Abstraction:** Udostępnia prosty interfejs (np. `set_gripper`)
    do kontrolowania aktywatorów, ukrywając szczegóły implementacyjne związane ze sterowaniem PWM.
    Provides a simple interface (e.g., `set_gripper`) to control actuators,
    hiding implementation details related to PWM control.
-   **Integracja z HardwareManager / HardwareManager Integration:** Współpracuje z `HardwareManager`,
    który jest odpowiedzialny za niskopoziomową komunikację ze sterownikami serwomechanizmów.
    Collaborates with `HardwareManager`, which is responsible for low-level communication
    with servo controllers.
-   **Obsługa wielu aktywatorów / Multi-actuator Support:** Zaprojektowany z myślą o łatwej rozbudowie
    o kolejne serwa i aktywatory w przyszłości.
    Designed for easy expansion with more servos and actuators in the future.
"""

import logging
from typing import TYPE_CHECKING

# Uniknięcie pętli importów - HardwareManager importuje ten moduł
# Avoid import loops - HardwareManager imports this module
if TYPE_CHECKING:
    from modules.managers.hardware_manager import HardwareManager


logger = logging.getLogger(__name__)


class ActuatorManager:
    """
    Menedżer aktywatorów i serwomechanizmów (Actuator Manager).
    Manages additional servos and actuators, e.g., gripper.
    """

    def __init__(self, hw_manager: "HardwareManager"):
        """
        Inicjalizuje ActuatorManager.
        Initializes the ActuatorManager.

        Args:
            hw_manager (HardwareManager): Referencja do głównego zarządcy sprzętu.
                                          Reference to the main hardware manager.
        """
        self.hw_manager = hw_manager
        logger.info("ActuatorManager zainicjalizowany.")

    def set_gripper(self, value: float) -> None:
        """
        Ustawia pozycję serwomechanizmu chwytaka.
        Sets the position of the gripper servo.

        Args:
            value (float): Wartość w zakresie od -1.0 (otwarty) do 1.0 (zamknięty).
                           Value ranging from -1.0 (open) to 1.0 (closed).
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
        """
        Czyści zasoby i zatrzymuje aktywatory (jeśli wymagane).
        Cleans up resources and stops actuators (if required).

        Zapewnia bezpieczne wyłączenie serwomechanizmów przy zamykaniu aplikacji.
        Ensures safe shutdown of servos when the application closes.
        """
        logger.info("ActuatorManager wykonuje cleanup.")
        # W tym prostym przypadku nie ma zasobów do zwolnienia
        # In this simple case, there are no resources to free
