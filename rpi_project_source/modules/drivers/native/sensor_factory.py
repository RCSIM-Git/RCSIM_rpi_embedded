# The MIT License (MIT)
# Copyright (c) 2024 Jules
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
# OR OTHER DEALINGS IN THE SOFTWARE.

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
    """
    A container for detected sensor drivers that provides standardized output.
    This class acts as a unified interface to a collection of heterogeneous
    sensors, normalizing their outputs into a single, consistent dictionary
    format.

    Kontener na wykryte sterowniki sensorów, który dostarcza ustandaryzowany
    format wyjściowy. Klasa ta działa jak zunifikowany interfejs do kolekcji
    różnorodnych sensorów, normalizując ich dane wyjściowe do jednego,
    spójnego formatu słownika.
    """

    def __init__(self) -> None:
        self.imu: Union[NativeMPU6050, NativeMPU9250] | None = None
        self.barometer: Union[NativeBMP180, NativeBMP280] | None = None
        self.magnetometer: NativeQMC5883L | None = None

        # Gyroscope stationary bias [rad/s]
        # Statyczny bias żyroskopu [rad/s]
        self.gyro_bias: dict[str, float] = {"gx": 0.0, "gy": 0.0, "gz": 0.0}

    def read_all(self) -> dict[str, float | None]:
        """
        Reads data from all detected sensors, converts it to standard SI units,
        and returns it in a unified dictionary format.

        Odczytuje dane ze wszystkich wykrytych sensorów, konwertuje je na
        standardowe jednostki SI i zwraca w zunifikowanym formacie słownika.

        -   **Acceleration:** g's -> m/s^2
        -   **Gyroscope:** degrees/s -> rad/s (with bias correction)
        -   **Pressure:** Pascals -> hectopascals (hPa)
        -   **Altitude:** Calculated from pressure using the international
            barometric formula.

        Returns:
            dict[str, float | None]: A dictionary containing all available
                                        sensor data. Keys are standardized
                                        (ax, ay, az, gx, gy, gz, mx, my, mz,
                                        temp, pressure, altitude).
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
        """
        Calculates the gyroscope's stationary bias by averaging a number of
        samples. This should be performed once while the sensor is completely
        still.

        Oblicza stacjonarny bias żyroskopu poprzez uśrednienie pewnej liczby
        próbek. Powinno to być wykonane jednorazowo, gdy czujnik jest
        całkowicie nieruchomy.

        Args:
            samples (int): The number of samples to average. Defaults to 200.
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
    """
    Detects, initializes, and manages all supported I2C sensors.
    Wykrywa, inicjalizuje i zarządza wszystkimi wspieranymi sensorami I2C.
    """

    def __init__(self, i2c_wrapper: I2CWrapper):
        self.i2c = i2c_wrapper
        self.imu_group = IMUGroup()
        self.logger = logging.getLogger(__name__)

    def detect_and_initialize(self) -> IMUGroup:
        """
        Scans the I2C bus for known device addresses and attempts to initialize
        a driver for each detected sensor.

        Skanuje magistralę I2C w poszukiwaniu znanych adresów urządzeń i
        próbuje zainicjalizować sterownik dla każdego wykrytego sensora.

        Returns:
            IMUGroup: An `IMUGroup` instance containing all successfully
                      initialized drivers.
        """
        devices = self._scan_bus()
        self.logger.info(f"Detected I2C addresses: {[hex(d) for d in devices]}")

        # --- IMU Detection (MPU-6050 or MPU-9250) at 0x68 ---
        if 0x68 in devices:
            try:
                # Differentiate by reading the WHO_AM_I register
                # Rozróżnienie przez odczytanie rejestru WHO_AM_I
                who_am_i = self.i2c.read_byte_data(0x68, 0x75)
                if who_am_i == 0x71:  # MPU-9250 Chip ID
                    self.imu_group.imu = NativeMPU9250(self.i2c)
                    self.logger.info("Detected and initialized MPU-9250.")
                elif who_am_i in [0x68, 0x70]:  # MPU-6050 reports 0x68 or 0x70
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
        """
        Scans the I2C bus for connected devices. A device is considered
        present if it acknowledges its address.

        Skanuje magistralę I2C w poszukiwaniu podłączonych urządzeń.
        Urządzenie jest uważane za obecne, jeśli potwierdzi swój adres.

        Returns:
            list[int]: A list of detected I2C addresses.
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
