"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import logging
from typing import Any


class GimbalStabilizer:
    """
    Klasa do stabilizacji 2-osiowego gimbala na podstawie danych z IMU.
    Używa prostego kontrolera proporcjonalnego do kompensacji przechyłu i pochylenia.

    Class for stabilizing a 2-axis gimbal based on IMU data.
    Uses a simple proportional controller to compensate for roll and pitch.
    """

    def __init__(self, config: dict[str, Any], logger: logging.Logger) -> None:
        """
        Inicjalizuje stabilizator gimbala.
        Initializes the gimbal stabilizer.

        Args:
            config (dict[str, Any]): Słownik konfiguracyjny. / Configuration dictionary.
            logger (logging.Logger): Instancja loggera. / Logger instance.
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
        """
        Mapuje wartość z jednego zakresu na drugi.
        Maps a value from one range to another.
        """
        if from_max == from_min:
            return to_min

        value = max(from_min, min(from_max, value))
        from_span = from_max - from_min
        to_span = to_max - to_min
        scaled_value = (value - from_min) / from_span
        return to_min + (scaled_value * to_span)

    def update(self, imu_data: dict[str, float]) -> dict[str, int] | None:
        """
        Oblicza nowe pozycje dla serw gimbala na podstawie danych z IMU.
        Calculates new positions for the gimbal servos based on IMU data.

        Args:
            imu_data (dict[str, float]): Słownik z danymi z IMU. / Dictionary with IMU data.

        Returns:
            dict[str, int] | None: Słownik z obliczonymi wartościami impulsów dla serw. / A dictionary with the calculated pulse values for the servos.
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
        """
        Zwraca kanały PCA9685 używane przez gimbal.
        Returns the PCA9685 channels used by the gimbal.
        """
        return {
            "pitch": self.config["pitch_channel"],
            "roll": self.config["roll_channel"],
        }
