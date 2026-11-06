"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Wykrywa sensory I2C, inicjalizuje sterowniki i ujednolica ich dane.

EN: Detects I2C sensors, initializes drivers, and standardizes their data.
"""

"""
Sensor Factory for automatic detection, initialization, and data standardization
of I2C sensors.
Fabryka Sensorów do automatycznej detekcji, inicjalizacji i standaryzacji danych
dla sensorów I2C.
"""

import logging
import math
import time
from typing import Union

from .native_bmp180 import NativeBMP180
from .native_bmp280 import NativeBMP280
from .native_i2c import I2CWrapper
from .native_mpu6050 import NativeMPU6050
from .native_mpu9250 import NativeMPU9250
from .native_qmc5883l import NativeQMC5883L

# Type alias for sensor driver instances for clarity.
# Alias typu dla instancji sterowników sensorów dla czytelności.
SensorDriver = Union[
    NativeMPU6050, NativeMPU9250, NativeBMP180, NativeBMP280, NativeQMC5883L
]


class IMUGroup:
    """PL: Łączy dostępne sterowniki IMU, barometru i magnetometru.

    EN: Combines available IMU, barometer, and magnetometer drivers.
    """

    def __init__(self) -> None:
        """PL: Inicjalizuje puste przypisania sensorów i zerowy bias żyroskopu.

        EN: Initializes empty sensor references and zero gyroscope bias.
        """
        self.imu: Union[NativeMPU6050, NativeMPU9250] | None = None
        self.barometer: Union[NativeBMP180, NativeBMP280] | None = None
        self.magnetometer: NativeQMC5883L | None = None

        # Gyroscope stationary bias [rad/s]
        # Statyczny bias żyroskopu [rad/s]
        self.gyro_bias: dict[str, float] = {"gx": 0.0, "gy": 0.0, "gz": 0.0}

    def read_all(self) -> dict[str, float | None]:
        """PL: Odczytuje sensory i scala pomiary w ujednoliconym słowniku SI.

        Przelicza przyspieszenie, żyroskop, ciśnienie i wysokość.

        Returns:
            Słownik dostępnych pomiarów z ustandaryzowanymi nazwami pól.

        EN: Reads the sensors and merges their measurements into a standard SI dictionary.

        Converts acceleration, gyroscope readings, pressure, and altitude.

        Returns:
            A dictionary of available readings with standardized field names.
        """
        data: dict[str, float | None] = {}

        # --- Read and process IMU data (Accel/Gyro/Mag from MPU-9250) ---
        if self.imu:
            imu_data = self.imu.read_scaled()
            # Convert acceleration from g's to m/s^2
            # Konwersja przyspieszenia z g na m/s^2
            data["ax"] = imu_data.get("ax", 0.0) * 9.80665
            data["ay"] = imu_data.get("ay", 0.0) * 9.80665
            data["az"] = imu_data.get("az", 0.0) * 9.80665
            # Apply calibration bias and convert gyroscope from deg/s to rad/s
            # Zastosuj bias kalibracyjny i przekonwertuj żyroskop z deg/s
            # na rad/s
            data["gx"] = math.radians(imu_data.get("gx", 0.0) - self.gyro_bias["gx"])
            data["gy"] = math.radians(imu_data.get("gy", 0.0) - self.gyro_bias["gy"])
            data["gz"] = math.radians(imu_data.get("gz", 0.0) - self.gyro_bias["gz"])
            # MPU-9250 includes its own magnetometer
            # MPU-9250 zawiera własny magnetometr
            if "mx" in imu_data:
                data["mx"] = imu_data["mx"]
                data["my"] = imu_data["my"]
                data["mz"] = imu_data["mz"]

        # --- Read standalone magnetometer if no MPU-9250 is present ---
        if self.magnetometer and "mx" not in data:
            mx, my, mz = self.magnetometer.read_scaled()
            data["mx"], data["my"], data["mz"] = mx, my, mz

        # --- Read and process Barometer data (Temp/Pressure/Altitude) ---
        if self.barometer:
            temp, pressure_pa = self.barometer.read_scaled()
            data["temp"] = temp
            # Convert pressure from Pascals (Pa) to hectopascals (hPa)
            # Konwersja ciśnienia z Pascali (Pa) na hektopaskale (hPa)
            pressure_hpa = pressure_pa / 100.0
            data["pressure"] = pressure_hpa
            # Calculate altitude using the international barometric formula,
            # assuming standard sea level pressure.
            # Oblicz wysokość używając międzynarodowej formuły
            # barometrycznej, zakładając standardowe ciśnienie na
            # poziomie morza.
            data["altitude"] = 44330.0 * (1.0 - pow(pressure_hpa / 1013.25, 0.1903))

        return data

    def calibrate_gyro(self, samples: int = 200) -> None:
        """PL: Oblicza bias żyroskopu przez uśrednienie próbek nieruchomego IMU.

        Args:
            samples: Liczba próbek do uśrednienia, domyślnie 200.

        EN: Calculates gyroscope bias by averaging samples from a stationary IMU.

        Args:
            samples: Number of samples to average, defaulting to 200.
        """
        if not self.imu:
            logging.warning("Gyro calibration skipped: No IMU detected.")
            return

        logging.info(
            f"Calibrating gyroscope with {samples} samples... "
            "Do not move the sensor."
        )
        sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0
        for _ in range(samples):
            raw = self.imu.read_scaled()
            sum_gx += raw.get("gx", 0.0)
            sum_gy += raw.get("gy", 0.0)
            sum_gz += raw.get("gz", 0.0)
            time.sleep(0.01)  # Wait between samples

        self.gyro_bias["gx"] = sum_gx / samples
        self.gyro_bias["gy"] = sum_gy / samples
        self.gyro_bias["gz"] = sum_gz / samples
        logging.info(
            f"Gyro bias calculated: gx={self.gyro_bias['gx']:.2f}, "
            f"gy={self.gyro_bias['gy']:.2f}, gz={self.gyro_bias['gz']:.2f} deg/s"
        )


class SensorManager:
    """PL: Wykrywa i inicjalizuje obsługiwane sensory na magistrali I2C.

    EN: Detects and initializes supported sensors on the I2C bus.
    """

    def __init__(self, i2c_wrapper: I2CWrapper):
        """PL: Przygotowuje managera do skanowania wskazanej magistrali I2C.

        EN: Prepares the manager to scan the specified I2C bus.
        """
        self.i2c = i2c_wrapper
        self.imu_group = IMUGroup()
        self.logger = logging.getLogger(__name__)

    def detect_and_initialize(self) -> IMUGroup:
        """PL: Wykrywa obsługiwane układy i inicjalizuje ich sterowniki.

        Returns:
            Grupę sensorów ze sterownikami, których inicjalizacja się powiodła.

        EN: Detects supported chips and initializes their drivers.

        Returns:
            A sensor group containing successfully initialized drivers.
        """
        devices = self._scan_bus()
        self.logger.info(f"Detected I2C addresses: {[hex(d) for d in devices]}")

        # --- IMU Detection (MPU-6050 or MPU-9250) at 0x68 ---
        if 0x68 in devices:
            try:
                # Differentiate by reading the WHO_AM_I register
                who_am_i = self.i2c.read_byte_data(0x68, 0x75)
                if who_am_i in [0x71, 0x73, 0x70]:  # MPU-9250, MPU-9255, MPU-6500 (GY-91 compatible)
                    self.imu_group.imu = NativeMPU9250(self.i2c)
                    self.logger.info(f"Detected and initialized MPU-9250/9255/6500 (ID: {hex(who_am_i)}).")
                elif who_am_i == 0x68:  # MPU-6050 reports 0x68
                    self.imu_group.imu = NativeMPU6050(self.i2c)
                    self.logger.info("Detected and initialized MPU-6050.")
            except (IOError, RuntimeError) as e:
                self.logger.error(f"Error initializing sensor at 0x68: {e}")

        # --- Barometer Detection (BMP series) at 0x77 or 0x76 ---
        baro_addr = next((addr for addr in [0x77, 0x76] if addr in devices), None)
        if baro_addr:
            try:
                # Differentiate by reading the Chip ID register (0xD0)
                # Rozróżnienie przez odczytanie rejestru Chip ID (0xD0)
                chip_id = self.i2c.read_byte_data(baro_addr, 0xD0)
                if chip_id == 0x58:  # BMP280 Chip ID
                    self.imu_group.barometer = NativeBMP280(self.i2c, address=baro_addr)
                    self.logger.info(
                        f"Detected and initialized BMP280 at {hex(baro_addr)}."
                    )
                elif chip_id == 0x55:  # BMP180 Chip ID
                    self.imu_group.barometer = NativeBMP180(self.i2c, address=baro_addr)
                    self.logger.info(
                        f"Detected and initialized BMP180 at {hex(baro_addr)}."
                    )
            except (IOError, RuntimeError) as e:
                self.logger.error(f"Error at {hex(baro_addr)}: {e}")

        # --- Standalone Magnetometer (QMC5883L) at 0x0D ---
        # Only initialize if we don't have an MPU-9250, which has its own
        # magnetometer.
        # Inicjalizuj tylko, jeśli nie mamy MPU-9250, który ma własny
        # magnetometr.
        if 0x0D in devices and not isinstance(self.imu_group.imu, NativeMPU9250):
            try:
                self.imu_group.magnetometer = NativeQMC5883L(self.i2c)
                self.logger.info("Detected and initialized QMC5883L.")
            except (IOError, RuntimeError) as e:
                self.logger.error(f"Error initializing sensor at 0x0D: {e}")

        return self.imu_group

    def _scan_bus(self) -> list[int]:
        """PL: Skanuje zakres adresów I2C i zbiera adresy odpowiadających urządzeń.

        Returns:
            Listę wykrytych adresów urządzeń.

        EN: Scans the I2C address range and collects addresses that acknowledge.

        Returns:
            A list of detected device addresses.
        """
        devices: list[int] = []
        # Standard I2C address range
        # Standardowy zakres adresów I2C
        for addr in range(0x03, 0x78):
            try:
                # The read operation will fail with an IOError if no device
                # ACKs the address.
                # Operacja odczytu zakończy się błędem IOError, jeśli żadne
                # urządzenie nie potwierdzi adresu.
                self.i2c.bus.read_byte(addr)
                devices.append(addr)
            except IOError:
                pass  # No device at this address
        return devices
