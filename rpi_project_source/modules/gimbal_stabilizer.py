"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Oblicza sygnały serw kompensujące przechył dwuosiowego gimbala.
EN: Computes servo signals that compensate roll and pitch for a two-axis gimbal.
"""
import logging
from typing import Any


class GimbalStabilizer:
    """PL: Stabilizuje gimbal na podstawie kątów odczytanych z IMU.
    EN: Stabilizes a gimbal using angles read from the IMU.
    """

    def __init__(self, config: dict[str, Any], logger: logging.Logger) -> None:
        """PL: Zapisuje konfigurację i sprawdza wymagane ustawienia osi.
        EN: Stores configuration and checks the required axis settings.
        """
        self.logger: logging.Logger = logger
        self.config: dict[str, Any] = config
        self.logger.info(
            "Stabilizator gimbala zainicjalizowany z konfiguracją: %s", config
        )

        required_keys = [
            "pitch_channel",
            "roll_channel",
            "pitch_min_pulse",
            "pitch_max_pulse",
            "roll_min_pulse",
            "roll_max_pulse",
            "pitch_min_angle",
            "pitch_max_angle",
            "roll_min_angle",
            "roll_max_angle",
            "p_gain",
        ]
        if not all(key in self.config for key in required_keys):
            raise ValueError("Konfiguracja gimbala jest niekompletna.")

    def _map_value(
        self,
        value: float,
        from_min: float,
        from_max: float,
        to_min: float,
        to_max: float,
    ) -> float:
        """PL: Ogranicza wartość wejściową i mapuje ją do zakresu wyjściowego.
        EN: Clamps an input value and maps it to the output range.
        """
        if from_max == from_min:
            return to_min

        value = max(from_min, min(from_max, value))
        from_span = from_max - from_min
        to_span = to_max - to_min
        scaled_value = (value - from_min) / from_span
        return to_min + (scaled_value * to_span)

    def update(self, imu_data: dict[str, float]) -> dict[str, int] | None:
        """PL: Oblicza impulsy serw kompensujące kąty pitch i roll z IMU.
        EN: Computes servo pulses that compensate the IMU pitch and roll angles.
        """
        pitch = imu_data.get("pitch")
        roll = imu_data.get("roll")

        if pitch is None or roll is None:
            self.logger.warning("Brak danych 'pitch' lub 'roll' w odczycie IMU.")
            return None

        target_pitch_angle = -pitch * self.config["p_gain"]
        target_roll_angle = -roll * self.config["p_gain"]

        pitch_pulse = self._map_value(
            target_pitch_angle,
            self.config["pitch_min_angle"],
            self.config["pitch_max_angle"],
            self.config["pitch_min_pulse"],
            self.config["pitch_max_pulse"],
        )

        roll_pulse = self._map_value(
            target_roll_angle,
            self.config["roll_min_angle"],
            self.config["roll_max_angle"],
            self.config["roll_min_pulse"],
            self.config["roll_max_pulse"],
        )

        return {"pitch_pulse": int(pitch_pulse), "roll_pulse": int(roll_pulse)}

    def get_channels(self) -> dict[str, int]:
        """PL: Zwraca numery kanałów PCA9685 przypisane do osi gimbala.
        EN: Returns the PCA9685 channel numbers assigned to the gimbal axes.
        """
        return {
            "pitch": self.config["pitch_channel"],
            "roll": self.config["roll_channel"],
        }
