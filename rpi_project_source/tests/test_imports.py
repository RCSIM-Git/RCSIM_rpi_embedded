import importlib

import pytest

# List of modules to test
# Lista modułów do przetestowania
MODULES_TO_TEST = [
    "modules.drivers.native.native_gy91",
    "modules.drivers.native.native_i2c",
    "modules.drivers.native.native_mpu6050",
    "modules.drivers.native.native_mpu9250",
    "modules.drivers.native.native_pca9685",
    "modules.drivers.native.native_qmc5883l",
    "modules.drivers.native.sensor_factory",
    "modules.drivers.native.ups",
    "modules.managers.hardware_manager",
    "modules.managers.slam_manager",
    "modules.planners.local_planner",
    "modules.drivers.lidar.ld08",
    "modules.drivers.base_sensor",
    "modules.drivers.imu.gy91",
    "modules.drivers.imu.imu_bmp388",
    "modules.drivers.imu.imu_bmx160",
    "modules.drivers.imu.imu_gy87",
    "modules.drivers.imu.imu_mpu9250",
    "core.main_service",
    "core.webrtc_manager",
    "core.udp_service",
    "core.safety_supervisor",
    "core.config_loader",
    "core.web_service",
]


@pytest.mark.parametrize("module_name", MODULES_TO_TEST)
def test_import_module(module_name):
    """
    Sprawdza, czy dany moduł importuje się poprawnie.
    Checks if the given module imports correctly.
    """
    try:
        importlib.import_module(module_name)
    except ImportError as e:
        pytest.fail(f"Could not import {module_name}: {e}")
    except Exception as e:
        pytest.fail(f"Exception during import of {module_name}: {e}")
