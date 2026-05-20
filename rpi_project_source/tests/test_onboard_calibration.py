"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License.
See LICENSE file in the project root for full license information.

Testy jednostkowe dla pokładowej kalibracji IMU w SensorAggregatorze.
Unit tests for onboard IMU calibration in SensorAggregator.
"""

import sys
from unittest.mock import MagicMock

import pytest

# Mockowanie modułów sprzętowych przed importem
# Mock hardware modules before import
hardware_modules = [
    "board",
    "busio",
    "smbus2",
    "serial",
    "adafruit_pca9685",
    "psutil",
    "adafruit_bno08x",
    "adafruit_bno08x.i2c",
]
for mod in hardware_modules:
    if mod not in sys.modules:
        sys.modules[mod] = MagicMock()

from modules.hardware.sensor_aggregator import SensorAggregator  # noqa: E402


def test_apply_imu_calibration():
    """Weryfikacja matematyczna nakładania kalibracji IMU pokładowo."""
    # Setup mock config
    config = {
        "imu": {"driver": "auto"},
        "imu_calibration": {
            "gyro_bias_x": 0.1,
            "gyro_bias_y": -0.2,
            "gyro_bias_z": 0.05,
            "accel_offset_x": 0.01,
            "accel_offset_y": -0.02,
            "accel_offset_z": 0.15,
            "accel_scale_x": 1.05,
            "accel_scale_y": 0.98,
            "accel_scale_z": 1.01,
            "mag_offset_x": 5.0,
            "mag_offset_y": -3.5,
            "mag_offset_z": 10.0,
            "mag_scale_x": 0.95,
            "mag_scale_y": 1.02,
            "mag_scale_z": 0.99,
        },
    }

    # Inicjalizacja z mockowanym I2CWrapper
    aggregator = SensorAggregator(config, i2c_bus=MagicMock())

    # Surowe dane z żyrosensorów
    raw_imu = {
        "ax": 1.0,
        "ay": -1.0,
        "az": 9.8,
        "gx": 0.5,
        "gy": -0.5,
        "gz": 0.0,
        "mx": 20.0,
        "my": -20.0,
        "mz": 30.0,
    }

    # Aplikacja kalibracji
    aggregator._apply_imu_calibration(raw_imu)

    # 1. Sprawdzenie flagi kalibracji
    assert raw_imu["calibrated"] is True

    # 2. Kalibracja żyroskopu: gx - bias = 0.5 - 0.1 = 0.4
    assert pytest.approx(raw_imu["gx"]) == 0.4
    assert pytest.approx(raw_imu["gy"]) == -0.3
    assert pytest.approx(raw_imu["gz"]) == -0.05

    # 3. Kalibracja akcelerometru:
    # (ax - offset) * scale = (1.0 - 0.01) * 1.05 = 0.99 * 1.05 = 1.0395
    assert pytest.approx(raw_imu["ax"]) == 1.0395
    assert pytest.approx(raw_imu["ay"]) == -0.9604
    assert pytest.approx(raw_imu["az"]) == 9.7465

    # 4. Kalibracja magnetometru:
    # (mx - offset) * scale = (20.0 - 5.0) * 0.95 = 15.0 * 0.95 = 14.25
    assert pytest.approx(raw_imu["mx"]) == 14.25
    assert pytest.approx(raw_imu["my"]) == -16.83
    assert pytest.approx(raw_imu["mz"]) == 19.8
