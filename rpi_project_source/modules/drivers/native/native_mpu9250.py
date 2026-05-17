"""
Native MPU-9250 9-DoF IMU Driver using a shared I2C wrapper.
Natywny sterownik 9-osiowego IMU MPU-9250 używający współdzielonego wrappera I2C.

This driver provides a low-level interface to the MPU-9250, which combines an
MPU-6050 (accelerometer/gyroscope) with an AK8963 (magnetometer) in a single package.

Ten sterownik dostarcza niskopoziomowy interfejs do MPU-9250, który łączy w sobie
MPU-6050 (akcelerometr/żyroskop) z AK8963 (magnetometr) w jednej obudowie.
"""

import logging
import time

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

from .native_ak8963 import NativeAK8963
from .native_i2c import I2CWrapper


@SensorRegistry.register
class NativeMPU9250(IMUBase):
    """
    A lightweight, direct-register-access driver for the MPU-9250 9-DoF IMU.
    Lekki sterownik z bezpośrednim dostępem do rejestrów dla 9-osiowego IMU
    MPU-9250.
    """

    DRIVER_NAME = "native_mpu9250"
    I2C_ADDRESSES = [0x68]
    PRIORITY = 30

    @classmethod
    def scan(cls, i2c) -> bool:
        """
        Sprawdza WHO_AM_I dla MPU-9250 / 9255 / 6500 / 6050 / 6000.
        WHO_AM_I check for MPU-9250 / 9255 / 6500 / 6050 / 6000.
        """
        try:
            who = i2c.read_byte_data(0x68, 0x75)
            return who in cls._COMPATIBLE_CHIP_IDS
        except (OSError, IOError):
            return False

    # --- Register Definitions ---
    _ADDRESS = 0x68
    _REG_WHO_AM_I = 0x75
    _CHIP_ID = 0x71  # Default MPU-9250
    # 🔧 FIX: Akceptuj też kompatybilne chipy / Accept compatible chips
    _COMPATIBLE_CHIP_IDS = [
        0x71,  # MPU-9250
        0x73,  # MPU-9255
        0x70,  # MPU-6500
        0x68,  # MPU-6050
        0x60,  # MPU-6000
    ]

    # Chipy posiadające (potencjalnie) magnetometr AK8963
    # Chips that (potentially) have AK8963 magnetometer
    _MAG_COMPATIBLE_IDS = [0x71, 0x73, 0x70]

    _CHIP_NAMES = {
        0x71: "MPU-9250 (9-DoF with magnetometer)",
        0x73: "MPU-9255 (9-DoF with magnetometer)",  # GY-91 chipsets
        0x70: "MPU-6500 (6-DoF, often rebranded MPU-9250)",
        0x68: "MPU-6050 (6-DoF, no magnetometer)",
        0x60: "MPU-6000/6500 (6-DoF, no magnetometer)",
    }

    _REG_USER_CTRL = 0x6A
    _REG_PWR_MGMT_1 = 0x6B
    _REG_INT_PIN_CFG = 0x37
    _REG_ACCEL_XOUT_H = 0x3B

    # --- Configuration Registers ---
    _REG_GYRO_CONFIG = 0x1B
    _REG_ACCEL_CONFIG = 0x1C
    _REG_PWR_MGMT_2 = 0x6C

    # --- Scale Factors (configurable based on range) ---
    # Accelerometer scale factors for different ranges
    _ACCEL_SCALES = {
        2: 16384.0,  # ±2g
        4: 8192.0,  # ±4g
        8: 4096.0,  # ±8g
        16: 2048.0,  # ±16g
    }
    # Gyroscope scale factors for different ranges
    _GYRO_SCALES = {
        250: 131.0,  # ±250°/s
        500: 65.5,  # ±500°/s
        1000: 32.8,  # ±1000°/s
        2000: 16.4,  # ±2000°/s
    }

    def __init__(
        self,
        i2c_wrapper: I2CWrapper,
        address: int = _ADDRESS,
        accel_range: int = 2,
        gyro_range: int = 250,
    ):
        """
        Initializes the MPU-9250 driver.
        Inicjalizuje sterownik MPU-9250.

        Args:
            i2c_wrapper (I2CWrapper): The shared I2C wrapper instance.
            address (int): The I2C address of the MPU-9250.
        """
        self.i2c = i2c_wrapper
        self.address = address
        self.magnetometer: NativeAK8963 | None = None

        # Configuration
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.accel_scale = self._ACCEL_SCALES[self.accel_range]
        self.gyro_scale = self._GYRO_SCALES[self.gyro_range]

        # Bias offsets
        self.accel_offset: list[float] = [0.0, 0.0, 0.0]
        self.gyro_offset: list[float] = [0.0, 0.0, 0.0]

        # [NEW] CompassMot Coefficients (Linear interference from motors)
        self.mag_coeff: list[float] = [0.0, 0.0, 0.0]

        # Verify we are communicating with the correct chip
        # Weryfikacja, czy komunikujemy się z właściwym układem
        chip_id = self.i2c.read_byte_data(self.address, self._REG_WHO_AM_I)
        if chip_id not in self._COMPATIBLE_CHIP_IDS:
            raise RuntimeError(
                "Invalid MPU chip ID: expected one of "
                f"{[hex(x) for x in self._COMPATIBLE_CHIP_IDS]}, got {hex(chip_id)}"
            )

        chip_name = self._CHIP_NAMES.get(
            chip_id, f"Unknown MPU variant ({hex(chip_id)})"
        )
        logging.info(f"✓ Detected IMU: {chip_name} at address {hex(self.address)}")

        # 🔧 FIX: Initialization sequence zgodny z ricardozago GY91
        # 1. Soft Reset
        self.reset()
        # 2. Wake up and select clock source
        self.wake_up()

        # 5. Set hardware ranges explicitly to match self.accel_range and
        # self.gyro_range
        self.set_accel_range(self.accel_range)
        self.set_gyro_range(self.gyro_range)

        # After enabling bypass mode, the AK8963 is accessible on the main
        # I2C bus (MPU-9250/9255 and rebranded MPU-6500)
        # Po włączeniu trybu bypass, AK8963 jest dostępny na głównej
        # magistrali I2C (MPU-9250/9255 i rebrandowane MPU-6500)

        # 🔧 FIX: Próbuj zainicjalizować magnetometr dla wariantów 9-DoF
        # (W MPU-6500 często występuje magnetometer jeśli jest to moduł GY-91)
        # Attempt to initialize magnetometer for 9-DoF variants
        # (MPU-6500 often has a magnetometer if part of GY-91 module)
        if chip_id in self._MAG_COMPATIBLE_IDS:
            try:
                # Small delay to ensure bypass bus is stable
                time.sleep(0.05)
                self.magnetometer = NativeAK8963(i2c_wrapper)
                logging.info("AK8963 magnetometer initialized successfully.")
            except (RuntimeError, IOError) as e:
                self.magnetometer = None
                # Log as info/warning depending on chip ID to reduce noise
                # for genuine MPU-6500
                if chip_id == 0x70:
                    logging.info(
                        "Could not initialize AK8963 on MPU-6500 "
                        f"(expected if genuine 6-DoF): {e}"
                    )
                else:
                    logging.warning(f"Could not initialize AK8963 magnetometer: {e}")
        else:
            # MPU-6050/6000 nie ma magnetometru
            self.magnetometer = None
            logging.info(
                f"{chip_name} does not have a magnetometer. "
                "Magnetometer data will be unavailable."
            )

    def reset(self) -> None:
        """
        Performs a soft reset of the device.
        Wykonuje miękki reset urządzenia.
        """
        logging.info("Resetting MPU device...")
        # Write 0x80 to PWR_MGMT_1 to trigger reset
        self.i2c.write_byte_data(self.address, self._REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)  # Wait for reset to complete

    def wake_up(self) -> None:
        """
        Wakes the device from sleep mode.
        Wybudza urządzenie z trybu uśpienia.
        """
        # 0x01 = Auto select best clock source (PLL if ready, else internal oscillator)
        self.i2c.write_byte_data(self.address, self._REG_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

    def activate_all_sensors(self) -> None:
        """
        Aktywuje wszystkie sensory (akcelerometr i żyroskop).
        Activates all sensors (accelerometer and gyroscope).

        🔧 Zgodność z kodem ricardozago: PWR_MGMT_2 = 0x00 włącza wszystkie osie.
        """
        # 0x00 = Enable all axes (accel + gyro)
        self.i2c.write_byte_data(self.address, self._REG_PWR_MGMT_2, 0x00)
        time.sleep(0.01)
        logging.info("✓ All MPU sensors activated")

    def enable_bypass_mode(self) -> None:
        """
        Enables I2C bypass mode. This is crucial for allowing the host
        (e.g., Raspberry Pi) to directly access the AK8963 magnetometer,
        which is on an auxiliary I2C bus internal to the MPU-9250.

        Włącza tryb I2C bypass. Jest to kluczowe, aby umożliwić hostowi
        (np. Raspberry Pi) bezpośredni dostęp do magnetometru AK8963,
        który znajduje się na pomocniczej magistrali I2C wewnętrznej
        dla MPU-9250.
        """
        # 1. Disable I2C Master Mode (Bit 5 of USER_CTRL)
        # Some chip states or previous drivers might have enabled it.
        # Bypass mode requires I2C_MST_EN to be 0.
        user_ctrl = self.i2c.read_byte_data(self.address, self._REG_USER_CTRL)
        if user_ctrl & 0x20:
            logging.info("Disabling I2C Master mode to allow Bypass.")
            self.i2c.write_byte_data(
                self.address, self._REG_USER_CTRL, user_ctrl & ~0x20
            )
            time.sleep(0.01)

        # 2. Setting the BYPASS_EN bit (bit 1) to 1
        # Ustawienie bitu BYPASS_EN (bit 1) na 1
        int_pin_cfg = self.i2c.read_byte_data(self.address, self._REG_INT_PIN_CFG)
        self.i2c.write_byte_data(
            self.address, self._REG_INT_PIN_CFG, int_pin_cfg | 0x02
        )
        time.sleep(0.05)

        # Verify if bypass bit was set
        val = self.i2c.read_byte_data(self.address, self._REG_INT_PIN_CFG)
        if val & 0x02:
            logging.info("I2C Bypass enabled successfully.")
        else:
            logging.warning(f"Failed to enable I2C Bypass! INT_PIN_CFG: {bin(val)}")

    def set_accel_range(self, range_g: int) -> None:
        """
        Sets the accelerometer full-scale range.
        Ustawia pełny zakres skali akcelerometru.

        Args:
            range_g (int): Range in g (2, 4, 8, 16).
        """
        if range_g not in self._ACCEL_SCALES:
            supported = list(self._ACCEL_SCALES.keys())
            raise ValueError(f"Invalid accel range: {range_g}. Supported: {supported}")

        # Map range to register value (bits 4:3)
        # 2g: 00 (0x00), 4g: 01 (0x08), 8g: 10 (0x10), 16g: 11 (0x18)
        ranges = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}
        reg_val = ranges[range_g]

        logging.info(f"Setting MPU accel range to ±{range_g}g (reg: {hex(reg_val)})")
        self.i2c.write_byte_data(self.address, self._REG_ACCEL_CONFIG, reg_val)
        self.accel_range = range_g
        self.accel_scale = self._ACCEL_SCALES[range_g]

    def set_gyro_range(self, range_dps: int) -> None:
        """
        Sets the gyroscope full-scale range.
        Ustawia pełny zakres skali żyroskopu.

        Args:
            range_dps (int): Range in degrees per second (250, 500, 1000, 2000).
        """
        if range_dps not in self._GYRO_SCALES:
            supported = list(self._GYRO_SCALES.keys())
            raise ValueError(f"Invalid gyro range: {range_dps}. Supported: {supported}")

        # Map range to register value (bits 4:3)
        # 250: 00 (0x00), 500: 01 (0x08), 1000: 10 (0x10), 2000: 11 (0x18)
        ranges = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}
        reg_val = ranges[range_dps]

        logging.info(f"Setting MPU gyro range to ±{range_dps}°/s (reg: {hex(reg_val)})")
        self.i2c.write_byte_data(self.address, self._REG_GYRO_CONFIG, reg_val)
        self.gyro_range = range_dps
        self.gyro_scale = self._GYRO_SCALES[range_dps]

    def _bytes_to_int(self, high: int, low: int) -> int:
        """
        Converts two bytes (high and low) to a signed 16-bit integer.
        Konwertuje dwa bajty (górny i dolny) na 16-bitową liczbę całkowitą ze
        znakiem.
        """
        value = (high << 8) | low
        return value if value < 32768 else value - 65536

    def read_scaled(self, throttle: float = 0.0) -> dict[str, float | None]:
        """
        Reads and scales all 9-DoF data to physical units.
        Odczytuje i skaluje wszystkie dane 9-DoF do jednostek fizycznych.

        - Accelerometer: g's
        - Gyroscope: degrees per second (°/s)
        - Magnetometer: microteslas (µT)

        Returns:
            dict[str, float | None]: A dictionary with standardized keys
                                        (ax, ay, az, gx, gy, gz, mx, my, mz).
                                        Magnetometer values will be None if
                                        initialization failed.
        """
        # Read accelerometer and gyroscope data in one block
        # Odczytaj dane akcelerometru i żyroskopu w jednym bloku
        mpu_data = self.i2c.read_i2c_block_data(
            self.address, self._REG_ACCEL_XOUT_H, 14
        )

        # --- Process Accelerometer and Gyroscope ---
        accel_x = (
            self._bytes_to_int(mpu_data[0], mpu_data[1]) / self.accel_scale
        ) - self.accel_offset[0]
        accel_y = (
            self._bytes_to_int(mpu_data[2], mpu_data[3]) / self.accel_scale
        ) - self.accel_offset[1]
        accel_z = (
            self._bytes_to_int(mpu_data[4], mpu_data[5]) / self.accel_scale
        ) - self.accel_offset[2]

        gyro_x = (
            self._bytes_to_int(mpu_data[8], mpu_data[9]) / self.gyro_scale
        ) - self.gyro_offset[0]
        gyro_y = (
            self._bytes_to_int(mpu_data[10], mpu_data[11]) / self.gyro_scale
        ) - self.gyro_offset[1]
        gyro_z = (
            self._bytes_to_int(mpu_data[12], mpu_data[13]) / self.gyro_scale
        ) - self.gyro_offset[2]

        # --- Process Magnetometer ---
        mag_x, mag_y, mag_z = None, None, None
        if self.magnetometer:
            try:
                # Type hint for read_scaled is tuple[float, float, float]
                # but read_scaled can return nans
                mx, my, mz = self.magnetometer.read_scaled()

                # [NEW] Apply Magnetic Compensation (CompassMot)
                if mx is not None:
                    mx -= throttle * self.mag_coeff[0]
                if my is not None:
                    my -= throttle * self.mag_coeff[1]
                if mz is not None:
                    mz -= throttle * self.mag_coeff[2]

                mag_x, mag_y, mag_z = mx, my, mz
            except IOError:
                # Handle cases where the magnetometer might disconnect
                mag_x, mag_y, mag_z = None, None, None

        # --- Combine into a standardized dictionary ---
        scaled_data = {
            "ax": accel_x,
            "ay": accel_y,
            "az": accel_z,
            "gx": gyro_x,
            "gy": gyro_y,
            "gz": gyro_z,
            "mx": mag_x,
            "my": mag_y,
            "mz": mag_z,
        }

        return scaled_data

    def read_data(self, throttle: float = 0.0) -> dict[str, float | None]:
        """
        Alias dla read_scaled() - zapewnia kompatybilność z interfejsem IMUBase.
        Alias for read_scaled() - provides compatibility with IMUBase interface.

        Args:
            throttle: Current throttle value for magnetic compensation.

        Returns:
            dict[str, float | None]: Dane IMU w formacie standardowym.
                                        / IMU data in standard format.
        """
        data = self.read_scaled(throttle=throttle)
        # Dodaj pole temperature jeśli nie istnieje (dla kompatybilności)
        if "temperature" not in data:
            data["temperature"] = 25.0  # Wartość domyślna / Default value
        return data

    def calibrate(self) -> bool:
        """
        Zbiera 500 próbek i oblicza średni bias dla akcelerometru i żyroskopu.
        Collects 500 samples and calculates mean bias for accel and gyro.
        """
        logging.info("MPU9250: Starting bias calibration (500 samples)...")
        num_samples = 500

        sum_acc = [0.0, 0.0, 0.0]
        sum_gyro = [0.0, 0.0, 0.0]

        # Temporarily reset offsets to get raw data
        old_acc_off = self.accel_offset
        old_gyro_off = self.gyro_offset
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]

        try:
            for _ in range(num_samples):
                data = self.read_scaled()
                # Assuming read_scaled returns dict with float values, handling None if necessary
                sum_acc[0] += data["ax"] if data["ax"] is not None else 0.0
                sum_acc[1] += data["ay"] if data["ay"] is not None else 0.0
                sum_acc[2] += data["az"] if data["az"] is not None else 0.0
                sum_gyro[0] += data["gx"] if data["gx"] is not None else 0.0
                sum_gyro[1] += data["gy"] if data["gy"] is not None else 0.0
                sum_gyro[2] += data["gz"] if data["gz"] is not None else 0.0
                time.sleep(0.005)  # ~200Hz sampling

            self.accel_offset = [
                sum_acc[0] / num_samples,
                sum_acc[1] / num_samples,
                (sum_acc[2] / num_samples) - 1.0,  # Remove gravity (1g) from Z
            ]
            self.gyro_offset = [
                sum_gyro[0] / num_samples,
                sum_gyro[1] / num_samples,
                sum_gyro[2] / num_samples,
            ]

            logging.info(
                "MPU9250: Calibration finished. "
                f"Accel Offset: {self.accel_offset}, "
                f"Gyro Offset: {self.gyro_offset}"
            )
            return True
        except Exception as e:
            logging.error(f"MPU9250: Calibration failed: {e}")
            self.accel_offset = old_acc_off
            self.gyro_offset = old_gyro_off
            return False


# Standardized driver alias
IMUDriver = NativeMPU9250
