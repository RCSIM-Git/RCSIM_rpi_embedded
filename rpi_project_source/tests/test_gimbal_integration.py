"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License.
See LICENSE file in the project root for full license information.

Testy integracyjne dla zintegrowanej stabilizacji gimbala (Plan B) na RPi.
Integration tests for integrated gimbal stabilization (Plan B) on RPi.
"""

import sys
import time
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
    "RPi",
    "RPi.GPIO",
]
for mod in hardware_modules:
    if mod not in sys.modules:
        sys.modules[mod] = MagicMock()

from core.main_service import TelemetryWorker  # noqa: E402
from modules.hardware.sensor_aggregator import SensorAggregator  # noqa: E402


def test_complementary_filter_math():
    """Weryfikacja matematyczna filtra komplementarnego w SensorAggregator."""
    config = {
        "imu": {"driver": "auto"},
    }
    aggregator = SensorAggregator(config, i2c_bus=MagicMock())

    # Inicjalizacja czasu
    aggregator._last_orient_time = time.time() - 0.05
    aggregator._pitch_est = 0.0
    aggregator._roll_est = 0.0

    # 1. Test z surowym przyspieszeniem grawitacyjnym w osi Z (płasko)
    raw_imu = {
        "ax": 0.0,
        "ay": 0.0,
        "az": 9.81,
        "gx": 0.0,
        "gy": 0.0,
    }
    aggregator._update_imu_orientation(raw_imu)
    assert raw_imu["pitch"] == 0.0
    assert raw_imu["roll"] == 0.0

    # 2. Test z przyspieszeniem na osi X (przechył pitch)
    # ax = 9.81, ay = 0.0, az = 0.0 -> pitch_acc = atan2(9.81, 0) = 90 stopni
    raw_imu_tilt = {
        "ax": 9.81,
        "ay": 0.0,
        "az": 0.0,
        "gx": 0.0,
        "gy": 0.0,
    }
    aggregator._update_imu_orientation(raw_imu_tilt)
    # Z racji współczynnika 0.02 dla akcelerometru: 0.98 * 0 + 0.02 * 90 = 1.8 stopnia
    assert pytest.approx(raw_imu_tilt["pitch"]) == 1.8
    assert raw_imu_tilt["roll"] == 0.0


def test_gimbal_absolute_mode():
    """Weryfikacja działania gimbala w trybie bezwzględnym (Absolute Mode)."""
    config = {
        "gimbal": {
            "enabled": True,
            "p_gain": 1.0,
            "pitch_channel": 4,
            "roll_channel": 5,
            "pitch_mode": "absolute",
            "roll_mode": "absolute",
            "pitch_max_angle": 45.0,
            "roll_max_angle": 45.0,
            "pitch_min_angle": -45.0,
            "roll_min_angle": -45.0,
            "pitch_min_pulse": 1000,
            "pitch_max_pulse": 2000,
            "roll_min_pulse": 1000,
            "roll_max_pulse": 2000,
        }
    }

    # Inicjalizacja wątku telemetrii (z mockowanymi usługami i hardware)
    worker = TelemetryWorker(config)
    worker.hw_manager = MagicMock()
    worker.actuator_worker = MagicMock()

    # Stan sensora: pojazd stoi prosto (pitch=0, roll=0)
    sensor_data = {
        "imu": {
            "pitch": 0.0,
            "roll": 0.0,
        }
    }

    # 1. Joystick w pozycji neutralnej (1500 us) -> gimbal w centrum
    worker.extra_channels_data = {4: 1500, 5: 1500}
    worker._update_gimbal(sensor_data)
    assert worker.extra_channels_data[4] == 1500
    assert worker.extra_channels_data[5] == 1500

    # 2. Maksymalne wychylenie joysticka (2000 us) -> gimbal pod pełnym kątem (2000 us)
    worker.extra_channels_data = {4: 2000, 5: 1000}
    worker._update_gimbal(sensor_data)
    assert worker.extra_channels_data[4] == 2000
    assert worker.extra_channels_data[5] == 1000

    # 3. Dodanie przechyłu pojazdu (stabilizacja) przy joysticku na środku
    # Pojazd ma pitch = 10 deg -> gimbal musi skompensować o -10 deg
    # Kąt -10 deg maps: -10 / 45 * 500 = -111 us od środka -> ~1389 us
    sensor_data["imu"]["pitch"] = 10.0
    worker.extra_channels_data = {4: 1500, 5: 1500}
    worker._update_gimbal(sensor_data)
    assert pytest.approx(worker.extra_channels_data[4], abs=2) == 1389


def test_gimbal_rate_mode():
    """Weryfikacja działania gimbala w trybie przyrostowym (Rate Mode)."""
    config = {
        "gimbal": {
            "enabled": True,
            "p_gain": 1.0,
            "pitch_channel": 4,
            "roll_channel": 5,
            "pitch_mode": "rate",
            "roll_mode": "rate",
            "pitch_speed_scale": 30.0,  # 30 stopni na sekundę przy max wychyleniu
            "roll_speed_scale": 30.0,
            "pitch_max_angle": 45.0,
            "roll_max_angle": 45.0,
            "pitch_min_angle": -45.0,
            "roll_min_angle": -45.0,
            "pitch_min_pulse": 1000,
            "pitch_max_pulse": 2000,
            "roll_min_pulse": 1000,
            "roll_max_pulse": 2000,
        }
    }

    worker = TelemetryWorker(config)
    worker.hw_manager = MagicMock()
    worker.actuator_worker = MagicMock()
    worker.LOOP_TIME = 0.05  # Krok 20Hz

    sensor_data = {
        "imu": {
            "pitch": 0.0,
            "roll": 0.0,
        }
    }

    # Inicjalizacja offsetów
    worker.manual_pitch_offset = 0.0
    worker.manual_roll_offset = 0.0

    # 1. Joystick na środku (1500 us) -> brak przyrostu, offset pozostaje 0.0
    worker.extra_channels_data = {4: 1500, 5: 1500}
    worker._update_gimbal(sensor_data)
    assert worker.manual_pitch_offset == 0.0
    assert worker.extra_channels_data[4] == 1500

    # 2. Joystick maksymalnie w górę (2000 us) na jeden krok pętli (0.05s)
    # deflection=1.0, speed=30.0, dt=0.05s -> d_offset = 1.0*30.0*0.05 = 1.5 deg
    worker.extra_channels_data = {4: 2000, 5: 1500}
    worker._update_gimbal(sensor_data)
    assert pytest.approx(worker.manual_pitch_offset) == 1.5
    # Kąt 1.5 maps: 1500 + 1.5 / 45 * 500 = ~1516 us
    assert pytest.approx(worker.extra_channels_data[4], abs=2) == 1516

    # 3. Joystick wraca na środek (1500 us) -> offset zatrzymuje się na 1.5 stopnia
    worker.extra_channels_data = {4: 1500, 5: 1500}
    worker._update_gimbal(sensor_data)
    assert pytest.approx(worker.manual_pitch_offset) == 1.5
    assert pytest.approx(worker.extra_channels_data[4], abs=2) == 1516

    # 4. Limit (clamping) w trybie przyrostowym
    # Ustawiamy offset blisko limitu (44.5 deg) i drążek na max (deflection=1.0)
    # Wychylenie powinno zatrzymać się dokładnie na 45.0 i nie przekroczyć go
    worker.manual_pitch_offset = 44.5
    worker.extra_channels_data = {4: 2000, 5: 1500}
    worker._update_gimbal(sensor_data)
    assert pytest.approx(worker.manual_pitch_offset) == 45.0
    assert worker.extra_channels_data[4] == 2000
