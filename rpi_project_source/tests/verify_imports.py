import os
import sys

# Add the project root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

modules_to_test = [
    "modules.drivers.native.native_gy91",
    "modules.drivers.native.native_i2c",
    "modules.drivers.native.native_mpu6050",
    "modules.drivers.native.native_mpu9250",
    "modules.drivers.native.native_pca9685",
    "modules.drivers.native.native_qmc5883l",
    "modules.drivers.native.sensor_factory",
    "modules.drivers.native.ups",
    "modules.managers.hardware_manager",
    "modules.managers.legacy.slam_manager",
    "modules.planners.local_planner",
    "modules.drivers.lidar.ld08",
    "modules.drivers.base_sensor",
    "modules.drivers.imu.gy91",
    "modules.drivers.imu.imu_bmp388",
    "modules.drivers.imu.imu_bmx160",
    "modules.drivers.imu.imu_gy87",
    "modules.drivers.imu.imu_mpu9250",
]

print("Verifying imports...")
failed = []
for module in modules_to_test:
    try:
        __import__(module)
        print(f"OK: {module}")
    except ImportError as e:
        print(f"FAILED: {module} - {e}")
        failed.append(module)
    except Exception as e:
        print(f"ERROR: {module} - {e}")
        failed.append(module)

if failed:
    print(f"\nVerification failed for {len(failed)} modules.")
    sys.exit(1)
else:
    print("\nAll imports successful.")
    sys.exit(0)
