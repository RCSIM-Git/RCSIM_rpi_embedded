"""
Natywny sterownik dla sensora 9-DoF BMX160 (BMI160 + BMM150).
Native driver for BMX160 (BMI160 + BMM150) 9-DoF Sensor.

Adaptacja kodu DFRobot_BMX160.py do struktury projektu.
Adapter from DFRobot_BMX160.py to project structure.
"""

import logging
import time
from typing import Any

import numpy as np
from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

from .native_i2c import I2CWrapper

logger: logging.Logger = logging.getLogger(__name__)


@SensorRegistry.register
class NativeBMX160(IMUBase):
    """
    Sterownik dla układu BMX160 integrującego akcelerometr, żyroskop i magnetometr.
    Zapewnia pełną zgodność z logiką DFRobot.
    Driver for the BMX160 chip integrating accelerometer, gyroscope, and magnetometer.
    Ensures full compatibility with DFRobot logic.
    """

    DRIVER_NAME = "native_bmx160"
    I2C_ADDRESSES = [0x68, 0x69]
    PRIORITY = 40

    _BMX160_CHIP_ID_ADDR = 0x00
    _BMX160_ERROR_REG_ADDR = 0x02
    _BMX160_MAG_DATA_ADDR = 0x04
    _BMX160_GYRO_DATA_ADDR = 0x0C
    _BMX160_ACCEL_DATA_ADDR = 0x12
    _BMX160_STATUS_ADDR = 0x1B
    _BMX160_INT_STATUS_ADDR = 0x1C
    _BMX160_FIFO_LENGTH_ADDR = 0x22
    _BMX160_FIFO_DATA_ADDR = 0x24
    _BMX160_ACCEL_CONFIG_ADDR = 0x40
    _BMX160_ACCEL_RANGE_ADDR = 0x41
    _BMX160_GYRO_CONFIG_ADDR = 0x42
    _BMX160_GYRO_RANGE_ADDR = 0x43
    _BMX160_MAGN_CONFIG_ADDR = 0x44
    _BMX160_FIFO_DOWN_ADDR = 0x45
    _BMX160_FIFO_CONFIG_0_ADDR = 0x46
    _BMX160_FIFO_CONFIG_1_ADDR = 0x47
    _BMX160_MAGN_RANGE_ADDR = 0x4B
    _BMX160_MAGN_IF_0_ADDR = 0x4C
    _BMX160_MAGN_IF_1_ADDR = 0x4D
    _BMX160_MAGN_IF_2_ADDR = 0x4E
    _BMX160_MAGN_IF_3_ADDR = 0x4F
    _BMX160_INT_ENABLE_0_ADDR = 0x50
    _BMX160_INT_ENABLE_1_ADDR = 0x51
    _BMX160_INT_ENABLE_2_ADDR = 0x52
    _BMX160_INT_OUT_CTRL_ADDR = 0x53
    _BMX160_INT_LATCH_ADDR = 0x54
    _BMX160_INT_MAP_0_ADDR = 0x55
    _BMX160_INT_MAP_1_ADDR = 0x56
    _BMX160_INT_MAP_2_ADDR = 0x57
    _BMX160_INT_DATA_0_ADDR = 0x58
    _BMX160_INT_DATA_1_ADDR = 0x59
    _BMX160_INT_LOWHIGH_0_ADDR = 0x5A
    _BMX160_INT_LOWHIGH_1_ADDR = 0x5B
    _BMX160_INT_LOWHIGH_2_ADDR = 0x5C
    _BMX160_INT_LOWHIGH_3_ADDR = 0x5D
    _BMX160_INT_LOWHIGH_4_ADDR = 0x5E
    _BMX160_INT_MOTION_0_ADDR = 0x5F
    _BMX160_INT_MOTION_1_ADDR = 0x60
    _BMX160_INT_MOTION_2_ADDR = 0x61
    _BMX160_INT_MOTION_3_ADDR = 0x62
    _BMX160_INT_TAP_0_ADDR = 0x63
    _BMX160_INT_TAP_1_ADDR = 0x64
    _BMX160_INT_ORIENT_0_ADDR = 0x65
    _BMX160_INT_ORIENT_1_ADDR = 0x66
    _BMX160_INT_FLAT_0_ADDR = 0x67
    _BMX160_INT_FLAT_1_ADDR = 0x68
    _BMX160_FOC_CONF_ADDR = 0x69
    _BMX160_CONF_ADDR = 0x6A
    _BMX160_IF_CONF_ADDR = 0x6B
    _BMX160_SELF_TEST_ADDR = 0x6D
    _BMX160_OFFSET_ADDR = 0x71
    _BMX160_OFFSET_CONF_ADDR = 0x77
    _BMX160_INT_STEP_CNT_0_ADDR = 0x78
    _BMX160_INT_STEP_CONFIG_0_ADDR = 0x7A
    _BMX160_INT_STEP_CONFIG_1_ADDR = 0x7B
    _BMX160_COMMAND_REG_ADDR = 0x7E

    BMX160_SOFT_RESET_CMD = 0xB6
    BMX160_MAGN_UT_LSB = 0.3
    _BMX160_ACCEL_MG_LSB_2G = 0.000061035
    _BMX160_ACCEL_MG_LSB_4G = 0.000122070
    _BMX160_ACCEL_MG_LSB_8G = 0.000244141
    _BMX160_ACCEL_MG_LSB_16G = 0.000488281

    _BMX160_GYRO_SENSITIVITY_125DPS = 0.0038110
    _BMX160_GYRO_SENSITIVITY_250DPS = 0.0076220
    _BMX160_GYRO_SENSITIVITY_500DPS = 0.0152439
    _BMX160_GYRO_SENSITIVITY_1000DPS = 0.0304878
    _BMX160_GYRO_SENSITIVITY_2000DPS = 0.0609756

    GyroRange_125DPS = 0x00
    GyroRange_250DPS = 0x01
    GyroRange_500DPS = 0x02
    GyroRange_1000DPS = 0x03
    GyroRange_2000DPS = 0x04

    AccelRange_2G = 0x00
    AccelRange_4G = 0x01
    AccelRange_8G = 0x02
    AccelRange_16G = 0x03

    def __init__(self, i2c_wrapper: I2CWrapper, address: int = 0x68) -> None:
        """
        Inicjalizuje sterownik BMX160.
        Initializes the BMX160 driver.
        """
        self.i2c: I2CWrapper = i2c_wrapper
        self.addr: int = address

        self.accelRange = self._BMX160_ACCEL_MG_LSB_2G
        self.gyroRange = self._BMX160_GYRO_SENSITIVITY_250DPS

        # Offsets calibration (zachowane z poprzedniej wersji dla kompatybilności z interfejsem IMUBase)
        # [NEW] CompassMot Coefficients (Linear interference from motors)
        # Applied as: m_clean = m_raw - (throttle * coeff)
        self.mag_coeff: list[float] = [0.0, 0.0, 0.0]  # Default 0

        # Inicjalizacja sprzętowa (begin) / Hardware initialization (begin)
        if not self.begin():
            logger.error("BMX160 initialization failed!")
            raise RuntimeError("BMX160 initialization failed!")
        else:
            logger.info("BMX160 initialized successfully.")

    def begin(self) -> bool:
        """!
        @brief initialization the i2c.
        @return returns the initialization status
        @retval True Initialization succeeded
        @retval False Initialization  failed
        """
        if not self.scan():
            return False
        else:
            self.soft_reset()
            self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x11)
            time.sleep(0.05)
            self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x15)
            time.sleep(0.1)
            self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x19)
            time.sleep(0.01)
            self.write_bmx_reg(self._BMX160_MAG_DATA_ADDR, 0x01)  # Test write
            self.set_magn_conf()

            # [PLAN-005] Vitality Check: Ensure sensor is actually alive and noisy
            if not self.check_vitality():
                logger.error("BMX160: Sensor data is flat (not alive/frozen)!")
                return False

            return True

    def set_low_power(self) -> None:
        """!
        @brief disabled the the magn, gyro sensor to reduce power consumption
        """
        self.soft_reset()
        time.sleep(0.1)
        self.set_magn_conf()
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x12)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x17)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x1B)
        time.sleep(0.1)

    def wake_up(self) -> None:
        """!
        @brief enabled the the magn, gyro sensor
        """
        self.soft_reset()
        time.sleep(0.1)
        self.set_magn_conf()
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x11)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x15)
        time.sleep(0.1)
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, 0x19)
        time.sleep(0.1)

    def soft_reset(self) -> bool:
        """!
        @brief reset bmx160 hardware
        @return returns the reset status
        @retval True reset succeeded
        @retval False reset  failed
        """
        data = self.BMX160_SOFT_RESET_CMD
        self.write_bmx_reg(self._BMX160_COMMAND_REG_ADDR, data)
        time.sleep(0.015)
        return True

    def set_magn_conf(self) -> None:
        """!
        @brief  set magnetometer Config
        """
        self.write_bmx_reg(self._BMX160_MAGN_IF_0_ADDR, 0x80)
        time.sleep(0.05)
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x01)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x4B)
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x04)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x51)
        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x0E)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x52)

        self.write_bmx_reg(self._BMX160_MAGN_IF_3_ADDR, 0x02)
        self.write_bmx_reg(self._BMX160_MAGN_IF_2_ADDR, 0x4C)
        self.write_bmx_reg(self._BMX160_MAGN_IF_1_ADDR, 0x42)
        self.write_bmx_reg(self._BMX160_MAGN_CONFIG_ADDR, 0x08)
        self.write_bmx_reg(self._BMX160_MAGN_IF_0_ADDR, 0x03)
        time.sleep(0.05)

    def set_gyro_range(self, bits: int) -> None:
        """!
        @brief set gyroscope angular rate range and resolution.
        @param bits
        @n       GyroRange_125DPS      Gyroscope sensitivity at 125dps
        @n       GyroRange_250DPS      Gyroscope sensitivity at 250dps
        @n       GyroRange_500DPS      Gyroscope sensitivity at 500dps
        @n       GyroRange_1000DPS     Gyroscope sensitivity at 1000dps
        @n       GyroRange_2000DPS     Gyroscope sensitivity at 2000dps
        """
        if bits == 0:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_125DPS
        elif bits == 1:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_250DPS
        elif bits == 2:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_500DPS
        elif bits == 3:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_1000DPS
        elif bits == 4:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_2000DPS
        else:
            self.gyroRange = self._BMX160_GYRO_SENSITIVITY_250DPS

    def set_accel_range(self, bits: int) -> None:
        """!
        @brief allow the selection of the accelerometer g-range.
        @param bits
        @n       AccelRange_2G        Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000061035mg)
        @n       AccelRange_4G        Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000122070mg)
        @n       AccelRange_8G        Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000244141mg)
        @n       AccelRange_16G       Macro for mg per LSB at +/- 16g sensitivity (1 LSB = 0.000488281mg)
        """
        if bits == 0:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_2G
        elif bits == 1:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_4G
        elif bits == 2:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_8G
        elif bits == 3:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_16G
        else:
            self.accelRange = self._BMX160_ACCEL_MG_LSB_2G

    def get_all_data(self, throttle: float = 0.0) -> list[float]:
        """!
        @brief get the magn, gyro and accel data
        @return all data
        """
        # DFRobot code reads 20 bytes starting from _BMX160_MAG_DATA_ADDR (0x04)
        data = self.read_bmx_reg(self._BMX160_MAG_DATA_ADDR, 20)

        # Guard against read failure
        if not data or len(data) < 20:
            logger.warning("BMX160: Read failed or incomplete data.")
            return [0.0] * 9

        if data[1] & 0x80:
            magnx = -0x10000 + ((data[1] << 8) | (data[0]))
        else:
            magnx = (data[1] << 8) | (data[0])
        if data[3] & 0x80:
            magny = -0x10000 + ((data[3] << 8) | (data[2]))
        else:
            magny = (data[3] << 8) | (data[2])
        if data[5] & 0x80:
            magnz = -0x10000 + ((data[5] << 8) | (data[4]))
        else:
            magnz = (data[5] << 8) | (data[4])

        if data[9] & 0x80:
            gyrox = -0x10000 + ((data[9] << 8) | (data[8]))
        else:
            gyrox = (data[9] << 8) | (data[8])
        if data[11] & 0x80:
            gyroy = -0x10000 + ((data[11] << 8) | (data[10]))
        else:
            gyroy = (data[11] << 8) | (data[10])
        if data[13] & 0x80:
            gyroz = -0x10000 + ((data[13] << 8) | (data[12]))
        else:
            gyroz = (data[13] << 8) | (data[12])

        if data[15] & 0x80:
            accelx = -0x10000 + ((data[15] << 8) | (data[14]))
        else:
            accelx = (data[15] << 8) | (data[14])
        if data[17] & 0x80:
            accely = -0x10000 + ((data[17] << 8) | (data[16]))
        else:
            accely = (data[17] << 8) | (data[16])
        if data[19] & 0x80:
            accelz = -0x10000 + ((data[19] << 8) | (data[18]))
        else:
            accelz = (data[19] << 8) | (data[18])

        magnx *= self.BMX160_MAGN_UT_LSB
        magny *= self.BMX160_MAGN_UT_LSB
        magnz *= self.BMX160_MAGN_UT_LSB

        gyrox *= self.gyroRange
        gyroy *= self.gyroRange
        gyroz *= self.gyroRange

        accelx *= self.accelRange * 9.8
        accely *= self.accelRange * 9.8
        accelz *= self.accelRange * 9.8

        # [NEW] Apply Magnetic Compensation (CompassMot)
        # Applied to RAW data before any conversion or EKF processing
        magnx -= throttle * self.mag_coeff[0]
        magny -= throttle * self.mag_coeff[1]
        magnz -= throttle * self.mag_coeff[2]

        out_put = []
        out_put.append(magnx)
        out_put.append(magny)
        out_put.append(magnz)
        out_put.append(gyrox)
        out_put.append(gyroy)
        out_put.append(gyroz)
        out_put.append(accelx)
        out_put.append(accely)
        out_put.append(accelz)
        return out_put

    def write_bmx_reg(self, register: int, value: int) -> None:
        """!
        @brief Write data to the BMX register
        @param register register
        @param value  Data written to the BMX register
        @return return the actually written length
        """
        self.i2c.write_byte_data(self.addr, register, value)

    def read_bmx_reg(self, register: int, length: int = 1) -> list[int]:
        """!
        @brief Read BMX register data
        @param register register
        @return data
        """
        return self.i2c.read_i2c_block_data(self.addr, register, length)

    def scan(self) -> bool:
        """!
        @brief  iic scan function
        @return scan result
        @retval True sensor exist
        @retval False There is no sensor
        """
        try:
            # Używamy read_byte_data(self.addr, 0x00) (Chip ID) jako testu obecności
            # We use read_byte_data(self.addr, 0x00) (Chip ID) as presence test
            chip_id = self.i2c.read_byte_data(self.addr, self._BMX160_CHIP_ID_ADDR)

            # BMI160/BMX160 usually return 0xD1, but some BMX160 return 0xD8
            if chip_id in [0xD1, 0xD8]:
                logger.info(
                    f"✓ Detected BMX160/BMI160 at address {hex(self.addr)} (ID: {hex(chip_id)})"
                )
                return True
            else:
                logger.warning(
                    f"BMX160 scan failed: Invalid Chip ID {hex(chip_id)} (expected 0xD1 or 0xD8)"
                )
                return False

        except Exception:
            logger.error("BMX160 I2C init fail")
            return False

    def check_vitality(self, samples: int = 10) -> bool:
        """
        Sprawdza czy sensor żyje (czy dane nie są płaskie).
        Checks if the sensor is alive (whether data is not flat).
        """
        logger.info(f"BMX160: Checking vitality ({samples} samples)...")
        test_data = []
        for _ in range(samples):
            test_data.append(self.get_all_data())
            time.sleep(0.01)

        test_np = np.array(test_data)
        # Obliczamy wariancję dla Accel (6,7,8) i Gyro (3,4,5)
        # We calculate variance for Accel and Gyro
        variances = np.var(test_np[:, 3:9], axis=0)

        # Jeśli jakakolwiek wariancja jest > 0, to sensor żyje
        # If any variance > 0, the sensor is alive
        if np.any(variances > 1e-9):
            logger.info(f"BMX160 Vitality OK (Var avg: {np.mean(variances):.2e})")
            return True

        return False

    # --- Metody zgodności (Compatibility Methods) ---

    def read_data(self, throttle: float = 0.0) -> dict[str, Any]:
        """
        Wrapper dla get_all_data() zapewniający zgodność z interfejsem IMUBase.
        Reads data from the sensor and returns it in a standardized format.
        """
        all_data = self.get_all_data(throttle=throttle)

        # Apply offsets (kalibracja software'owa)
        # Note: DFRobot code does not seem to apply offsets in get_all_data,
        # but we iterate on self.acc_offset/gyro_offset inside calibrate().
        # We apply them here for the final output.

        mx, my, mz = all_data[0], all_data[1], all_data[2]
        gx, gy, gz = all_data[3], all_data[4], all_data[5]
        ax, ay, az = all_data[6], all_data[7], all_data[8]

        return {
            "ax": ax - self.acc_offset[0],
            "ay": ay - self.acc_offset[1],
            "az": az - self.acc_offset[2],
            "gx": gx - self.gyro_offset[0],
            "gy": gy - self.gyro_offset[1],
            "gz": gz - self.gyro_offset[2],
            "mx": mx,
            "my": my,
            "mz": mz,
            "temperature": 25.0,  # Placeholder, DFRobot code doesn't read temp
        }

    def calibrate(self) -> bool:
        """
        Zbiera próbki i oblicza średni bias dla akcelerometru i żyroskopu.
        Uses get_all_data() to collect samples.
        """
        logger.info("BMX160: Starting bias calibration (200 samples)...")
        num_samples = 200

        sum_acc = [0.0, 0.0, 0.0]
        sum_gyro = [0.0, 0.0, 0.0]

        try:
            for _ in range(num_samples):
                # get_all_data zwraca surowe/przeskalowane dane bez offsetu
                # get_all_data returns raw/scaled data without offset
                data_list = self.get_all_data()

                # Accel (indeksy 6,7,8)
                sum_acc[0] += data_list[6]
                sum_acc[1] += data_list[7]
                sum_acc[2] += data_list[8]

                # Gyro (indeksy 3,4,5)
                sum_gyro[0] += data_list[3]
                sum_gyro[1] += data_list[4]
                sum_gyro[2] += data_list[5]

                time.sleep(0.01)

            avg_acc = [sum_acc[i] / num_samples for i in range(3)]
            avg_gyro = [sum_gyro[i] / num_samples for i in range(3)]

            # Kalibracja:
            # Zakładamy że robot stoi płasko: Z = 9.8 (lub 1g * 9.8), X=0, Y=0.
            # DFRobot zwraca m/s^2 w get_all_data (mnoży przez 9.8).
            # Więc oczekujemy Z = 9.8.
            # Calibration:
            # Assume robot is flat: Z = 9.8, X=0, Y=0.
            # DFRobot returns m/s^2.

            self.acc_offset = [avg_acc[0] - 0.0, avg_acc[1] - 0.0, avg_acc[2] - 9.8]
            self.gyro_offset = avg_gyro

            logger.info(
                f"BMX160: Calibration finished. Avg Acc: {avg_acc}, Offsets saved (Z-9.8)."
            )
            return True
        except Exception as e:
            logger.error(f"BMX160: Calibration failed: {e}")
            return False
