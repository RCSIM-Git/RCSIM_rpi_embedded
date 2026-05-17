"""
Biblioteka matematyczna - Kinematyka Ackermanna.
Mathematical library - Ackermann Kinematics.

Przelicza przesunięcie przestrzenne na układzie współrzędnych bez bycia wpiętym
w logikę zapytań sprzętowych I2C/UART modułów RPi.
"""

import math
from typing import Any


def estimate_motion(
    throttle: float,
    steering: float,
    dt: float,
    max_speed_mps: float = 2.0,
    max_steer_rad: float = 0.52,
    wheelbase_m: float = 0.25,
    imu_data: dict[str, Any] | None = None,
) -> tuple[float, float, float]:
    """
    Estymuje przesunięcie robota (dx, dy, dyaw) na podstawie wejść sterujących i IMU.
    Estimates the robot's motion (dx, dy, dyaw) based on control inputs and IMU.

    Args:
        throttle (float): Wartość przepustnicy [-1.0, 1.0].
        steering (float): Wartość skrętu [-1.0, 1.0].
        dt (float): Czas od ostatniej aktualizacji [s].
        max_speed_mps (float): Maksymalna prędkość w metrach na sekundę.
        max_steer_rad (float): Maksymalne wychylenie promienia osi zwrotnicy w radianach (Ackermann).
        wheelbase_m (float): Rozstaw osi w metrach.
        imu_data (dict[str, Any] | None): Opcjonalne najnowsze odczyty sprzętowe z IMU w celu lepszej fuzji kątowej.

    Returns:
        tuple[float, float, float]: (dx, dy, dyaw) przesunięcie lokalne w metrach i radianach.
    """
    v = throttle * max_speed_mps
    steer_angle = steering * max_steer_rad

    # Uproszczony model kinematyczny Ackermanna
    # Simplified Ackermann kinematic model
    # dx = v * cos(yaw) * dt, dy = v * sin(yaw) * dt
    # Tutaj wyliczamy lokalne przesunięcie (dx, dy) względem robota przed ruchem
    dx = v * dt
    dy = 0.0

    # dyaw - Priorytet dla danych z Żyroskopu
    # Priority for Gyro data
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
        safe_wheelbase = max(0.001, wheelbase_m)
        # Limit steer angle to safe range for tan() [-85, 85] degrees
        safe_steer = max(-1.48, min(1.48, steer_angle))

        dyaw = (v / safe_wheelbase) * math.tan(safe_steer) * dt

    return dx, dy, dyaw
