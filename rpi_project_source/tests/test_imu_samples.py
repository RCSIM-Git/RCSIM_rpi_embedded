"""Measured-only IMU acquisition and MAVLink SI telemetry regressions."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from core.mavlink_service import MAVLinkService
from modules.hardware.sensor_aggregator import SensorAggregator


def aggregator(driver, result=None, failure=None):
    reader = Mock(return_value=result, side_effect=failure)
    obj = SensorAggregator.__new__(SensorAggregator)
    obj.config = {}
    obj.imu = SimpleNamespace(DRIVER_NAME=driver, read_data=reader)
    obj.gps = obj.lidar = obj.ups = None
    obj.last_throttle = 0
    obj._notified_failures = set()
    return obj


@pytest.mark.parametrize("driver,az", [("native_mpu9250", 1), ("native_bmx160", 9.81)])
def test_measured_units_and_acquisition_clock(driver, az):
    obj = aggregator(driver, dict(ax=0, ay=0, az=az, gx=180, gy=0, gz=0))
    first = obj.read_sensors()["imu"]
    second = obj.read_sensors()["imu"]
    assert first["az"] == pytest.approx(9.81)
    assert first["gx"] == pytest.approx(math.pi)
    assert first["gyro_unit"] == "rad/s"
    assert first["source"] == "rpi_raw"
    assert second["sample_time_us"] >= first["sample_time_us"]
    assert second["sample_sequence"] == first["sample_sequence"] + 1
    assert "mx" not in first


@pytest.mark.parametrize("result,failure", [(None, None), ({}, None), (None, OSError("bus failed")),
                                          (dict(ax=float("nan"), ay=0, az=1), None)])
def test_read_failure_has_no_synthetic_imu(result, failure):
    obj = aggregator("native_mpu9250", result, failure)
    assert obj.read_sensors()["imu"] is None


def test_absent_gyro_not_invented():
    obj = aggregator("native_bno08x", dict(ax=0, ay=0, az=1))
    imu = obj.read_sensors()["imu"]
    assert "gx" not in imu
    assert "roll" not in imu


def test_calibration_applied_in_canonical_units():
    obj = aggregator("native_mpu9250", dict(ax=0, ay=0, az=1, gx=180, gy=0, gz=0))
    obj.config["imu_calibration"] = {"gyro_bias_x": math.pi, "accel_offset_z": 0.81,
                                     "accel_unit": "m/s2", "gyro_unit": "rad/s"}
    imu = obj.read_sensors()["imu"]
    assert imu["gx"] == pytest.approx(0)
    assert imu["az"] == pytest.approx(9)


def test_legacy_calibration_keeps_native_driver_units():
    obj = aggregator("native_mpu9250", dict(ax=1, ay=0, az=1, gx=10, gy=0, gz=0))
    obj.config["imu_calibration"] = {"gyro_bias_x": 5, "accel_offset_x": 0.1}
    imu = obj.read_sensors()["imu"]
    assert imu["gx"] == pytest.approx(math.radians(5))
    assert imu["ax"] == pytest.approx(0.9 * 9.81)


def test_highres_si_and_missing_magnetometer_flags():
    service = MAVLinkService()
    service.master = SimpleNamespace(mav=Mock())
    service.send_attitude(0, 0, 0, dict(ax=0, ay=0, az=9.81, gx=math.pi, gy=0, gz=0,
                                     accel_unit="m/s2", gyro_unit="rad/s", sample_time_us=123))
    values = service.master.mav.highres_imu_send.call_args.args
    assert values[0] == 123
    assert values[3] == 9.81
    assert values[4] == math.pi
    assert values[-1] == 0b111111
    calls = service.master.mav.mock_calls
    assert calls[0][0] == "named_value_int_send"
    assert calls[0].args[1:] == (b"RCSIM_RPI", 1)
    service.master.mav.raw_imu_send.assert_not_called()
