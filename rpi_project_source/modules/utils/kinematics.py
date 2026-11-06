"""PL: Szacuje ruch robota z wejść sterujących modelem kinematyki Ackermanna.
EN: Estimates robot motion from control inputs using Ackermann kinematics.
"""

import math
from typing import Any, NamedTuple


class ControlInput(NamedTuple):
    """PL: Przechowuje wartości gazu i skrętu dla modelu Ackermanna.
    EN: Stores throttle and steering values for the Ackermann model.
    """
    throttle: float
    steering: float


class KinematicConfig(NamedTuple):
    """PL: Określa maksymalną prędkość, skręt i rozstaw osi modelu.
    EN: Defines the model's maximum speed, steering angle, and wheelbase.
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
    """PL: Szacuje lokalne przesunięcie i obrót z gazu, skrętu oraz opcjonalnego IMU.
    EN: Estimates local translation and rotation from throttle, steering, and optional IMU data.
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
