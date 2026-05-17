# rpi_project_source/tests/rpi/drivers/test_native_drivers.py
import os
import struct
import sys
from unittest.mock import MagicMock, patch

import pytest
from modules.drivers.native.native_bmp280 import NativeBMP280
from modules.drivers.native.native_i2c import I2CWrapper
from modules.drivers.native.native_mpu9250 import NativeMPU9250
from modules.drivers.native.native_pca9685 import NativePCA9685
from modules.drivers.native.sensor_factory import SensorManager

# Add project root to the Python path to allow imports from modules
project_root = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")
)
sys.path.insert(0, project_root)


@pytest.fixture
def mock_i2c_wrapper():
    """Fixture to create a mocked I2CWrapper that simulates the real one."""
    return MagicMock(spec=I2CWrapper)


def test_bmp280_read_data_with_datasheet_values(mock_i2c_wrapper):
    """
    Testuje, czy sterownik NativeBMP280 poprawnie oblicza temperaturę i ciśnienie,
    używając danych referencyjnych z noty katalogowej Bosch.
    """
    # Arrange
    mock_i2c_wrapper.read_byte_data.return_value = 0x58  # BMP280 Chip ID

    # Dane kalibracyjne z noty katalogowej BMP280 (sekcja 3.11.3)
    calib_values = {
        "T1": 27504,
        "T2": 26435,
        "T3": -1000,
        "P1": 36477,
        "P2": -10685,
        "P3": 3024,
        "P4": 2855,
        "P5": 140,
        "P6": -7,
        "P7": 15500,
        "P8": -14600,
        "P9": 6000,
    }
    calib_data_bytes = struct.pack(
        "<HhhHhhhhhhhh",
        calib_values["T1"],
        calib_values["T2"],
        calib_values["T3"],
        calib_values["P1"],
        calib_values["P2"],
        calib_values["P3"],
        calib_values["P4"],
        calib_values["P5"],
        calib_values["P6"],
        calib_values["P7"],
        calib_values["P8"],
        calib_values["P9"],
    )

    # Surowe, 20-bitowe dane pomiarowe z noty katalogowej
    raw_temp = 519888
    raw_pressure = 415148

    # Poprawna programowa konwersja surowych danych na 6-bajtowy format,
    # jaki zwraca czujnik: [P_MSB, P_LSB, P_XLSB, T_MSB, T_LSB, T_XLSB]
    raw_sensor_data = [
        (raw_pressure >> 12) & 0xFF,
        (raw_pressure >> 4) & 0xFF,
        (raw_pressure << 4) & 0xF0,
        (raw_temp >> 12) & 0xFF,
        (raw_temp >> 4) & 0xFF,
        (raw_temp << 4) & 0xF0,
    ]

    mock_i2c_wrapper.read_i2c_block_data.side_effect = [
        list(calib_data_bytes),
        raw_sensor_data,
    ]

    # Act
    driver = NativeBMP280(mock_i2c_wrapper)
    temp, pressure = driver.read_scaled()

    # Assert
    # Oczekiwane wartości na podstawie noty katalogowej
    expected_temp = 25.08
    expected_pressure = 100000.0

    assert temp == pytest.approx(expected_temp, abs=0.01)
    assert pressure == pytest.approx(expected_pressure, abs=700.0)


def test_sensor_factory_detection_logic(mock_i2c_wrapper):
    """
    Test a SensorManager poprawnie wykrywa i inicjalizuje sterowniki.
    """
    # Arrange
    factory = SensorManager(mock_i2c_wrapper)
    dummy_calib_data = [0] * 24
    mock_i2c_wrapper.read_i2c_block_data.return_value = dummy_calib_data

    with patch.object(factory, "_scan_bus", return_value=[0x68, 0x77, 0x76]):

        def read_byte_data_side_effect(address, register):
            if address == 0x68 and register == 0x75:
                return 0x71
            if address == 0x77 and register == 0xD0:
                return 0x58
            if address == 0x76:
                raise OSError("No device")
            return 0x00

        mock_i2c_wrapper.read_byte_data.side_effect = read_byte_data_side_effect

        # Act
        imu_group = factory.detect_and_initialize()

        # Assert
        assert isinstance(imu_group.imu, NativeMPU9250)
        assert isinstance(imu_group.barometer, NativeBMP280)


def test_pca9685_set_pwm_logic(mock_i2c_wrapper):
    """
    Testuje, czy sterownik NativePCA9685 poprawnie oblicza i zapisuje
    wartości PWM do odpowiednich rejestrów.
    """
    # Arrange
    mock_i2c_wrapper.read_byte_data.return_value = 0x10
    driver = NativePCA9685(mock_i2c_wrapper, address=0x40)

    # Act
    driver.set_us(0, 1500)

    # Assert
    expected_off_tick = 307
    expected_data = [0, 0, expected_off_tick & 0xFF, expected_off_tick >> 8]

    mock_i2c_wrapper.write_i2c_block_data.assert_called_with(0x40, 0x06, expected_data)
