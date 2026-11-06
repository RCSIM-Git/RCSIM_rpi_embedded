"""PL: Testy rejestru sterowników czujników IMU.

EN: Tests for the IMU sensor driver registry.
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
    """PL: Symuluje czujnik o wysokim priorytecie wykrywania.

    EN: Simulate a sensor with a high detection priority.
    """

    DRIVER_NAME = "fake_high_priority"
    I2C_ADDRESSES = [0x50]
    PRIORITY = 10

    def read_data(self):
        """PL: Zwraca przykładowe dane przyspieszenia.

        EN: Return sample acceleration data.
        """
        return {"ax": 0.0}

    def calibrate(self):
        """PL: Symuluje pomyślną kalibrację czujnika.

        EN: Simulate successful sensor calibration.
        """
        return True


class FakeSensorLow(IMUBase):
    """PL: Symuluje czujnik o niskim priorytecie wykrywania.

    EN: Simulate a sensor with a low detection priority.
    """

    DRIVER_NAME = "fake_low_priority"
    I2C_ADDRESSES = [0x51]
    PRIORITY = 90

    def read_data(self):
        """PL: Zwraca przykładowe dane przyspieszenia.

        EN: Return sample acceleration data.
        """
        return {"ax": 0.0}

    def calibrate(self):
        """PL: Symuluje pomyślną kalibrację czujnika.

        EN: Simulate successful sensor calibration.
        """
        return True


class FakeSensorScan(IMUBase):
    """PL: Symuluje czujnik z własną procedurą wykrywania.

    EN: Simulate a sensor with a custom detection procedure.
    """

    DRIVER_NAME = "fake_custom_scan"
    I2C_ADDRESSES = [0x68]
    PRIORITY = 50

    @classmethod
    def scan(cls, i2c) -> bool:
        """PL: Wykrywa czujnik na podstawie rejestru WHO_AM_I.

        EN: Detect the sensor by checking its WHO_AM_I register.
        """
        try:
            who = i2c.read_byte_data(0x68, 0x75)
            return who == 0xAB
        except (OSError, IOError):
            return False

    def read_data(self):
        """PL: Zwraca przykładowe dane przyspieszenia.

        EN: Return sample acceleration data.
        """
        return {"ax": 0.0}

    def calibrate(self):
        """PL: Symuluje pomyślną kalibrację czujnika.

        EN: Simulate successful sensor calibration.
        """
        return True


@pytest.fixture(autouse=True)
def clean_registry():
    """PL: Czyści rejestr przed testem i odtwarza jego poprzedni stan.

    EN: Clear the registry before a test and restore its previous state.
    """
    original = SensorRegistry._registry.copy()
    SensorRegistry._registry.clear()
    yield
    SensorRegistry._registry = original


@pytest.fixture
def mock_i2c():
    """PL: Tworzy mock magistrali I2C do testów wykrywania.

    EN: Create a mocked I2C bus for detection tests.
    """
    i2c = MagicMock()
    i2c.read_byte_data = MagicMock(return_value=0x00)
    return i2c


# --- Testy / Tests ---


def test_register_adds_to_registry():
    """PL: Sprawdza dodawanie klasy sterownika do rejestru.

    EN: Verify that a driver class is added to the registry.
    """
    SensorRegistry.register(FakeSensorHigh)
    assert FakeSensorHigh in SensorRegistry._registry


def test_register_is_idempotent():
    """PL: Sprawdza, że ponowna rejestracja nie tworzy duplikatu.

    EN: Verify that repeated registration does not create duplicates.
    """
    SensorRegistry.register(FakeSensorHigh)
    SensorRegistry.register(FakeSensorHigh)
    count = SensorRegistry._registry.count(FakeSensorHigh)
    assert count == 1


def test_registry_sorted_by_priority():
    """PL: Sprawdza kolejność rejestru według priorytetu sterowników.

    EN: Verify registry ordering by driver priority.
    """
    SensorRegistry.register(FakeSensorLow)
    SensorRegistry.register(FakeSensorHigh)
    assert SensorRegistry._registry[0] is FakeSensorHigh
    assert SensorRegistry._registry[1] is FakeSensorLow


def test_detect_returns_highest_priority(mock_i2c):
    """PL: Sprawdza wybór wykrytego czujnika o najwyższym priorytecie.

    EN: Verify that detection selects the highest-priority matching sensor.
    """
    SensorRegistry.register(FakeSensorLow)
    SensorRegistry.register(FakeSensorHigh)

    # Oba sensory "wykryte" na I2C (read_byte_data nie rzuca)
    result = SensorRegistry.detect(mock_i2c)
    assert result is FakeSensorHigh


def test_detect_returns_none_when_no_match(mock_i2c):
    """PL: Sprawdza wynik braku dopasowania czujnika.

    EN: Verify the result when no sensor matches.
    """
    SensorRegistry.register(FakeSensorHigh)

    # Symulujemy brak urządzenia na I2C
    mock_i2c.read_byte_data.side_effect = OSError("No device")
    result = SensorRegistry.detect(mock_i2c)
    assert result is None


def test_get_by_name_found():
    """PL: Sprawdza pobieranie zarejestrowanego sterownika po nazwie.

    EN: Verify retrieving a registered driver by name.
    """
    SensorRegistry.register(FakeSensorHigh)
    SensorRegistry.register(FakeSensorLow)

    cls = SensorRegistry.get_by_name("fake_high_priority")
    assert cls is FakeSensorHigh


def test_get_by_name_not_found():
    """PL: Sprawdza wynik wyszukiwania nieznanej nazwy sterownika.

    EN: Verify the lookup result for an unknown driver name.
    """
    SensorRegistry.register(FakeSensorHigh)

    cls = SensorRegistry.get_by_name("nonexistent_driver")
    assert cls is None


def test_custom_scan_with_who_am_i(mock_i2c):
    """PL: Sprawdza wykrywanie przez niestandardowy skan WHO_AM_I.

    EN: Verify detection through a custom WHO_AM_I scan.
    """
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
    """PL: Sprawdza pobieranie listy wszystkich sterowników.

    EN: Verify retrieval of all registered drivers.
    """
    SensorRegistry.register(FakeSensorHigh)
    SensorRegistry.register(FakeSensorLow)

    drivers = SensorRegistry.all_drivers()
    assert len(drivers) == 2
    assert FakeSensorHigh in drivers
    assert FakeSensorLow in drivers


def test_base_sensor_scan_default(mock_i2c):
    """PL: Sprawdza domyślną metodę wykrywania IMUBase.

    EN: Verify the default IMUBase detection method.
    """
    # FakeSensorHigh ma adres 0x50, domyślny scan działa
    mock_i2c.read_byte_data.return_value = 0x00
    assert FakeSensorHigh.scan(mock_i2c) is True

    # Brak urządzenia
    mock_i2c.read_byte_data.side_effect = OSError()
    assert FakeSensorHigh.scan(mock_i2c) is False
