# rpi_project_source/tests/rpi/drivers/test_native_drivers.py
import os
import struct
import sys
from unittest.mock import MagicMock, patch

import pytest
from modules.drivers.native.native_ak8963 import NativeAK8963
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


def test_ak8963_magnetometer_logic(mock_i2c_wrapper):
    """
    Testuje, czy sterownik NativeAK8963 prawidłowo konfiguruje tryby bit_output
    oraz poprawnie oblicza odczyty skalowane.
    """
    # Arrange
    # 1. Zwróć poprawny Who Am I (0x48)
    def read_byte_side_effect(addr, reg):
        if reg == 0x00:  # _WIA
            return 0x48
        return 0

    mock_i2c_wrapper.read_byte_data.side_effect = read_byte_side_effect

    # 2. Zwróć fabryczne korekcje Fuse ROM (np. 128 dla braku korekcji)
    # _ASAX (0x10), długość 3
    mock_i2c_wrapper.read_i2c_block_data.return_value = [128, 128, 128]

    # Act
    driver = NativeAK8963(mock_i2c_wrapper)

    # Inicjalizacja domyślnie wywołuje set_mode() (Continuous 2 i 16-bit)
    # Sprawdźmy, czy CNTL1 (0x0A) zapisał 0x16 (0b00010110 = 100Hz + 16-bit)
    mock_i2c_wrapper.write_byte_data.assert_any_call(driver.address, 0x0A, 0x16)

    # Przetestujmy set_mode dla 14-bit
    mock_i2c_wrapper.write_byte_data.reset_mock()
    driver.set_mode(mode=0x06, bit_output=0)
    mock_i2c_wrapper.write_byte_data.assert_called_once_with(driver.address, 0x0A, 0x06)

    # Przetestujmy odczyty skalowane dla 16-bit
    driver.set_mode(mode=0x06, bit_output=1)
    
    # Symulujemy odczyt 7 bajtów z _HXL (0x03)
    # Surowe dane: x=1000, y=-500, z=2000, ST2=0 (brak overflow)
    # low_byte, high_byte
    # 1000 = 0x03E8 -> low=0xE8, high=0x03
    # -500 = 65036 = 0xFE0C -> low=0x0C, high=0xFE
    # 2000 = 0x07D0 -> low=0xD0, high=0x07
    raw_bytes = [0xE8, 0x03, 0x0C, 0xFE, 0xD0, 0x07, 0x00]
    mock_i2c_wrapper.read_i2c_block_data.return_value = raw_bytes

    mx, my, mz = driver.read_scaled()
    
    # Czułość dla 128 wynosi 1.0. Rozdzielczość 16-bit to 4912 / 32760.0
    res_16 = 4912.0 / 32760.0
    assert mx == pytest.approx(1000 * res_16)
    assert my == pytest.approx(-500 * res_16)
    assert mz == pytest.approx(2000 * res_16)

    # Przetestujmy odczyty dla 14-bit
    driver.set_mode(mode=0x06, bit_output=0)
    # Przy 14-bit np. dla raw 1000 rozdzielczość wynosi 4912 / 8190.0
    res_14 = 4912.0 / 8190.0
    mx, my, mz = driver.read_scaled()
    assert mx == pytest.approx(1000 * res_14)
    assert my == pytest.approx(-500 * res_14)
    assert mz == pytest.approx(2000 * res_14)


def test_mpu9250_compatible_chip_ids(mock_i2c_wrapper):
    """
    Testuje, czy NativeMPU9250 poprawnie akceptuje wszystkie kompatybilne identyfikatory chipów
    oraz odrzuca nieznane/nieobsługiwane identyfikatory.
    """
    # Lista wszystkich kompatybilnych chipów do przetestowania
    compatible_ids = [0x71, 0x73, 0x70, 0x68, 0x60]
    
    for chip_id in compatible_ids:
        mock_i2c_wrapper.read_byte_data.reset_mock()
        mock_i2c_wrapper.read_byte_data.return_value = chip_id
        
        # Inicjalizacja powinna przejść bez błędu RuntimeError dla kompatybilnego chipu
        driver = NativeMPU9250(mock_i2c_wrapper)
        assert driver.address == 0x68
        
    # Test nieznanego chip ID - powinien rzucić RuntimeError
    mock_i2c_wrapper.read_byte_data.return_value = 0xAA
    with pytest.raises(RuntimeError) as excinfo:
        NativeMPU9250(mock_i2c_wrapper)
    assert "Invalid MPU chip ID" in str(excinfo.value)


def test_sensor_factory_handles_mpu9255_and_6500(mock_i2c_wrapper):
    """
    Testuje, czy fabryka SensorManager poprawnie wykrywa i inicjalizuje NativeMPU9250
    dla chipów MPU-9255 (0x73) oraz MPU-6500 (0x70).
    """
    factory = SensorManager(mock_i2c_wrapper)
    dummy_calib_data = [0] * 24
    mock_i2c_wrapper.read_i2c_block_data.return_value = dummy_calib_data

    # Test dla MPU-9255 (0x73)
    with patch.object(factory, "_scan_bus", return_value=[0x68]):
        mock_i2c_wrapper.read_byte_data.return_value = 0x73  # MPU-9255
        imu_group = factory.detect_and_initialize()
        assert isinstance(imu_group.imu, NativeMPU9250)

    # Test dla MPU-6500 (0x70)
    with patch.object(factory, "_scan_bus", return_value=[0x68]):
        mock_i2c_wrapper.read_byte_data.return_value = 0x70  # MPU-6500
        imu_group = factory.detect_and_initialize()
        assert isinstance(imu_group.imu, NativeMPU9250)


def test_mpu9250_enable_bypass_mode(mock_i2c_wrapper):
    """
    Testuje, czy NativeMPU9250 poprawnie włącza tryb bypass na magistrali I2C
    podczas swojej inicjalizacji, co jest wymagane do komunikacji z AK8963.
    """
    # 1. Zwróć MPU-9250 (0x71) dla rejestru WHO_AM_I (0x75)
    # 2. Zwróć I2C Master jako włączony (0x20) dla USER_CTRL (0x6A), aby przetestować wyłączenie go
    # 3. Zwróć INT_PIN_CFG (0x37) jako 0x00, a potem 0x02 dla weryfikacji włączenia bypassu
    def read_byte_side_effect(addr, reg):
        if reg == 0x75:  # WHO_AM_I
            return 0x71
        if reg == 0x6A:  # USER_CTRL
            return 0x20  # I2C Master active
        if reg == 0x37:  # INT_PIN_CFG
            return 0x02  # Bypass enabled (for check)
        return 0

    mock_i2c_wrapper.read_byte_data.side_effect = read_byte_side_effect
    
    # Inicjalizujemy driver
    driver = NativeMPU9250(mock_i2c_wrapper)

    # Sprawdzamy, czy nastąpiło wyłączenie I2C Master (zapis 0 na bicie 5 USER_CTRL)
    mock_i2c_wrapper.write_byte_data.assert_any_call(driver.address, 0x6A, 0x00)

    # Sprawdzamy, czy ustawiono bit BYPASS_EN w INT_PIN_CFG (zapis 0x02 do 0x37)
    mock_i2c_wrapper.write_byte_data.assert_any_call(driver.address, 0x37, 0x02)



