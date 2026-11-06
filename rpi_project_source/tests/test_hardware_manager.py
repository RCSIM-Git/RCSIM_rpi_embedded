"""PL: Testy konfiguracji i inicjalizacji menedżera sprzętu.

EN: Tests for hardware manager configuration and initialization.
"""

import pytest
from modules.managers.hardware_manager import HardwareManager


@pytest.fixture
def hardware_config():
    """PL: Udostępnia minimalną konfigurację sprzętu do testów.

    EN: Provide a minimal hardware configuration for tests.
    """
    return {
        "i2c": {"bus": 1},
        "pca9685": {"frequency": 50},
        "motors": {
            "steering": {"channel": 0, "center_pwm": 1500, "range_pwm": [1000, 2000]},
            "throttle": {"channel": 1, "center_pwm": 1500, "range_pwm": [1000, 2000]},
        },
        "imu": {"enabled": False},
        "gps": {"enabled": False},
        "lidar": {"enabled": False},
    }


def test_hardware_manager_init(hardware_config, mock_rpi_hardware):
    """PL: Sprawdza inicjalizację menedżera z zamockowanym sprzętem.

    EN: Verify manager initialization with mocked hardware.
    """
    try:
        hw = HardwareManager(hardware_config, init_pca_neutral=False)
        assert hw is not None, "HardwareManager should be initialized"
        assert hw.pca is not None, "PCA9685 driver should be initialized (mocked)"
    except Exception as e:
        pytest.fail(f"HardwareManager init failed: {e}")


def test_hardware_manager_channels(hardware_config, mock_rpi_hardware):
    """PL: Sprawdza przypisanie kanałów i zakresu sterowania.

    EN: Verify channel assignments and control range.
    """
    hw = HardwareManager(hardware_config, init_pca_neutral=False)
    assert hw.steering_channel == 0
    assert hw.throttle_channel == 1
    assert hw.steering_range == (1000, 2000)
