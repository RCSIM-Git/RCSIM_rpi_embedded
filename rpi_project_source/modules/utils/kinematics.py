"""
Biblioteka matematyczna - Kinematyka Ackermanna.
Mathematical library - Ackermann Kinematics.

Przelicza przesunięcie przestrzenne na układzie współrzędnych bez bycia wpiętym
w logikę zapytań sprzętowych I2C/UART modułów RPi.
"""

import math
from typing import Any, NamedTuple


class ControlInput(NamedTuple):
    """
    Ackermann control input values.
    Wartości wejściowe sterowania Ackermanna.
    """
    throttle: float
    steering: float


class KinematicConfig(NamedTuple):
    """
    Ackermann kinematics configuration.
    Konfiguracja kinematyki Ackermanna.
    """
    max_speed_mps: float = 2.0
    max_steer_rad: float = 0.52
    wheelbase_m: float = 0.25


def estimate_motion(
    control: ControlInput,
    dt: float,
    config: KinematicConfig = KinematicConfig(),
    imu_data: dict[str, Any] | None = None,
) -> tuple[float, float, float]:
    """
    Estymuje przesunięcie robota (dx, dy, dyaw) na podstawie wejść sterujących i IMU.
    Estimates the robot's motion (dx, dy, dyaw) based on control inputs and IMU.

    Args:
        control (ControlInput): Wejścia sterujące (przepustnica, skręt). / Control inputs (throttle, steering).
        dt (float): Czas od ostatniej aktualizacji [s]. / Time since last update [s].
        config (KinematicConfig): Konfiguracja parametrów kinematycznych. / Kinematics configuration.
        imu_data (dict[str, Any] | None): Opcjonalne najnowsze odczyty sprzętowe z IMU. / Optional IMU data.

    Returns:
        tuple[float, float, float]: (dx, dy, dyaw) przesunięcie lokalne w metrach i radianach.
    """
    v = control.throttle * config.max_speed_mps
    steer_angle = control.steering * config.max_steer_rad

    # Uproszczony model kinematyczny Ackermanna
    # Simplified Ackermann kinematic model
    # dx = v * cos(yaw) * dt, dy = v * sin(yaw) * dt
    # Tutaj wyliczamy lokalne przesunięcie (dx, dy) względem robota przed ruchem
    dx = v * dt
    dy = 0.0

    # dyaw - Priorytet dla danych z Żyroskopu
    # Priority for Gyro data
    dyaw = 0.0
    if imu_data and "gz" in imu_data and abs(imu_data["gz"]) > 0.01:
        # Uwaga: Całkowanie błędu żyroskopu powoduje dryf (Gyroscopic Drift).
        # BreezySLAM koryguje to poprzez dopasowanie skanów (Particle Filter).
        dyaw = math.radians(imu_data["gz"]) * dt
    else:
        # dyaw = (v / L) * tan(steer_angle) * dt (Model Ackermanna)
        # [PLAN-004] Robustness: Protection against wheelbase=0 and singular tan()
        safe_wheelbase = max(0.001, config.wheelbase_m)
        # Limit steer angle to safe range for tan() [-85, 85] degrees
        safe_steer = max(-1.48, min(1.48, steer_angle))

        dyaw = (v / safe_wheelbase) * math.tan(safe_steer) * dt

    return dx, dy, dyaw
