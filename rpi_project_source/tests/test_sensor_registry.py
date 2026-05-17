"""
Testy dla SensorRegistry - Registry Pattern dla sensorów IMU.
Tests for SensorRegistry - Registry Pattern for IMU sensors.
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


from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

# --- Fixtury / Fixtures ---


class FakeSensorHigh(IMUBase):
    """Fake sensor — priorytet 10."""

    DRIVER_NAME = "fake_high_priority"
    I2C_ADDRESSES = [0x50]
    PRIORITY = 10

    def read_data(self):
        return {"ax": 0.0}

    def calibrate(self):
        return True


class FakeSensorLow(IMUBase):
    """Fake sensor — priorytet 90."""

    DRIVER_NAME = "fake_low_priority"
    I2C_ADDRESSES = [0x51]
    PRIORITY = 90

    def read_data(self):
        return {"ax": 0.0}

    def calibrate(self):
        return True


class FakeSensorScan(IMUBase):
    """Fake sensor z custom scan()."""

    DRIVER_NAME = "fake_custom_scan"
    I2C_ADDRESSES = [0x68]
    PRIORITY = 50

    @classmethod
    def scan(cls, i2c) -> bool:
        """Sprawdza WHO_AM_I == 0xAB."""
        try:
            who = i2c.read_byte_data(0x68, 0x75)
            return who == 0xAB
        except (OSError, IOError):
            return False

    def read_data(self):
        return {"ax": 0.0}

    def calibrate(self):
        return True


@pytest.fixture(autouse=True)
def clean_registry():
    """Czyści rejestr przed każdym testem."""
    original = SensorRegistry._registry.copy()
    SensorRegistry._registry.clear()
    yield
    SensorRegistry._registry = original


@pytest.fixture
def mock_i2c():
    """Mockowany I2CWrapper."""
    i2c = MagicMock()
    i2c.read_byte_data = MagicMock(return_value=0x00)
    return i2c


# --- Testy / Tests ---


def test_register_adds_to_registry():
    """@register dodaje klasę do rejestru."""
    SensorRegistry.register(FakeSensorHigh)
    assert FakeSensorHigh in SensorRegistry._registry


def test_register_is_idempotent():
    """Podwójna rejestracja nie duplikuje wpisu."""
    SensorRegistry.register(FakeSensorHigh)
    SensorRegistry.register(FakeSensorHigh)
    count = SensorRegistry._registry.count(FakeSensorHigh)
    assert count == 1


def test_registry_sorted_by_priority():
    """Rejestr sortowany wg priorytetu (niższy = pierwszy)."""
    SensorRegistry.register(FakeSensorLow)
    SensorRegistry.register(FakeSensorHigh)
    assert SensorRegistry._registry[0] is FakeSensorHigh
    assert SensorRegistry._registry[1] is FakeSensorLow


def test_detect_returns_highest_priority(mock_i2c):
    """detect() zwraca sensor z najwyższym priorytetem."""
    SensorRegistry.register(FakeSensorLow)
    SensorRegistry.register(FakeSensorHigh)

    # Oba sensory "wykryte" na I2C (read_byte_data nie rzuca)
    result = SensorRegistry.detect(mock_i2c)
    assert result is FakeSensorHigh


def test_detect_returns_none_when_no_match(mock_i2c):
    """detect() zwraca None gdy żaden sensor nie pasuje."""
    SensorRegistry.register(FakeSensorHigh)

    # Symulujemy brak urządzenia na I2C
    mock_i2c.read_byte_data.side_effect = OSError("No device")
    result = SensorRegistry.detect(mock_i2c)
    assert result is None


def test_get_by_name_found():
    """get_by_name() zwraca prawidłowy driver."""
    SensorRegistry.register(FakeSensorHigh)
    SensorRegistry.register(FakeSensorLow)

    cls = SensorRegistry.get_by_name("fake_high_priority")
    assert cls is FakeSensorHigh


def test_get_by_name_not_found():
    """get_by_name() zwraca None dla nieznanej nazwy."""
    SensorRegistry.register(FakeSensorHigh)

    cls = SensorRegistry.get_by_name("nonexistent_driver")
    assert cls is None


def test_custom_scan_with_who_am_i(mock_i2c):
    """Sensor z custom scan() korzysta z WHO_AM_I."""
    SensorRegistry.register(FakeSensorScan)

    # WHO_AM_I pasuje
    mock_i2c.read_byte_data.return_value = 0xAB
    result = SensorRegistry.detect(mock_i2c)
    assert result is FakeSensorScan

    # WHO_AM_I nie pasuje
    mock_i2c.read_byte_data.return_value = 0x00
    result = SensorRegistry.detect(mock_i2c)
    assert result is None


def test_all_drivers():
    """all_drivers() zwraca listę zarejestrowanych."""
    SensorRegistry.register(FakeSensorHigh)
    SensorRegistry.register(FakeSensorLow)

    drivers = SensorRegistry.all_drivers()
    assert len(drivers) == 2
    assert FakeSensorHigh in drivers
    assert FakeSensorLow in drivers


def test_base_sensor_scan_default(mock_i2c):
    """Domyślna implementacja scan() w IMUBase."""
    # FakeSensorHigh ma adres 0x50, domyślny scan działa
    mock_i2c.read_byte_data.return_value = 0x00
    assert FakeSensorHigh.scan(mock_i2c) is True

    # Brak urządzenia
    mock_i2c.read_byte_data.side_effect = OSError()
    assert FakeSensorHigh.scan(mock_i2c) is False
