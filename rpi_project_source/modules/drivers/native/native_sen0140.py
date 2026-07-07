"""
Połączony sterownik natywny dla modułu DFRobot SEN0140 (10-DoF IMU).
Combined native driver for the DFRobot SEN0140 module (10-DoF IMU).

Ten moduł zawiera implementację niskopoziomowego odczytu danych dla:
- ADXL345 (Akcelerometr)
- ITG3205 (Żyroskop)
- VCM5883L / HMC5883L / QMC5883L (Magnetometr)
- Oraz importuje BMP280 / BMP180 (Barometr)

Większość sensorów znajduje się na odrębnych adresach I2C, w przeciwieństwie
do zintegrowanych chipów jak MPU9250.
"""

import logging
from typing import Any

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry

try:
    from .native_bmp180 import NativeBMP180
    from .native_bmp280 import NativeBMP280
    from .native_i2c import I2CWrapper
    from .native_qmc5883l import NativeQMC5883L
except ImportError:
    from native_bmp180 import NativeBMP180
    from native_bmp280 import NativeBMP280
    from native_i2c import I2CWrapper
    from native_qmc5883l import NativeQMC5883L

logger: logging.Logger = logging.getLogger(__name__)


class NativeADXL345:
    """Sterownik akcelerometru ADXL345."""
    
    _DEFAULT_ADDRESS = 0x53
    _REG_POWER_CTL = 0x2D
    _REG_DATA_FORMAT = 0x31
    _REG_DATAX0 = 0x32

    def __init__(self, i2c: I2CWrapper, address: int = _DEFAULT_ADDRESS) -> None:
        self.i2c = i2c
        self.address = address
        
        # Test connection (read device ID 0x00 -> should be 0xE5)
        dev_id = self.i2c.read_byte_data(self.address, 0x00)
        if dev_id != 0xE5:
            raise RuntimeError(f"ADXL345 not found at {hex(address)}, ID: {hex(dev_id)}")
            
        # Initialize
        # +/- 16g, 13-bit full resolution
        self.i2c.write_byte_data(self.address, self._REG_DATA_FORMAT, 0x0B)
        # Put into measurement mode
        self.i2c.write_byte_data(self.address, self._REG_POWER_CTL, 0x08)
        
        # Scale factor for 16g full res is 3.9 mg/LSB -> 0.0039 * 9.80665 m/s^2
        self.scale = 0.0039 * 9.80665
        
    def read_scaled(self) -> tuple[float, float, float]:
        data = self.i2c.read_i2c_block_data(self.address, self._REG_DATAX0, 6)
        
        # ADXL345 is Little Endian
        x = (data[1] << 8) | data[0]
        y = (data[3] << 8) | data[2]
        z = (data[5] << 8) | data[4]
        
        # Convert to signed 16-bit
        x = x - 65536 if x >= 32768 else x
        y = y - 65536 if y >= 32768 else y
        z = z - 65536 if z >= 32768 else z
        
        return x * self.scale, y * self.scale, z * self.scale


class NativeITG3205:
    """Sterownik żyroskopu ITG3205 (ITG3200)."""
    
    _DEFAULT_ADDRESS = 0x68
    _REG_DLPF_FS = 0x16
    _REG_INT_CFG = 0x17
    _REG_PWR_MGM = 0x3E
    _REG_TEMP_OUT_H = 0x1B
    
    def __init__(self, i2c: I2CWrapper, address: int = _DEFAULT_ADDRESS) -> None:
        self.i2c = i2c
        self.address = address
        
        # Who am I? (Wait, ITG3205 who_am_i is at 0x00, contains I2C address)
        # Typically 0x68. Bits 1-6 contain address.
        who_am_i = self.i2c.read_byte_data(self.address, 0x00)
        
        # Set DLPF & Full Scale (+/- 2000 deg/s)
        self.i2c.write_byte_data(self.address, self._REG_DLPF_FS, 0x18)
        # Clock source = X gyro (better stability)
        self.i2c.write_byte_data(self.address, self._REG_PWR_MGM, 0x01)
        
        # Scale for ITG3205 is 14.375 LSB/(deg/s) -> 1/14.375 = 0.06956 deg/s per LSB
        # We need rad/s: (0.06956 * pi) / 180 = ~0.001214
        self.scale = 0.001214142

    def read_scaled(self) -> tuple[float, float, float]:
        data = self.i2c.read_i2c_block_data(self.address, self._REG_TEMP_OUT_H, 8)
        
        # ITG3205 is Big Endian
        # temp = (data[0] << 8) | data[1] (Ignoring temp for now as BMP is better)
        x = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]
        z = (data[6] << 8) | data[7]
        
        # Convert to signed 16-bit
        x = x - 65536 if x >= 32768 else x
        y = y - 65536 if y >= 32768 else y
        z = z - 65536 if z >= 32768 else z
        
        return x * self.scale, y * self.scale, z * self.scale


class NativeHMC5883L:
    """Sterownik magnetometru HMC5883L / VCM5883L."""
    
    _DEFAULT_ADDRESS = 0x1E
    _REG_CONFIG_A = 0x00
    _REG_CONFIG_B = 0x01
    _REG_MODE = 0x02
    _REG_DATA_OUT = 0x03
    
    def __init__(self, i2c: I2CWrapper, address: int = _DEFAULT_ADDRESS) -> None:
        self.i2c = i2c
        self.address = address
        
        # 8-average, 15 Hz, normal measurement
        self.i2c.write_byte_data(self.address, self._REG_CONFIG_A, 0x70)
        # Gain +/- 1.3 Ga
        self.i2c.write_byte_data(self.address, self._REG_CONFIG_B, 0x20)
        # Continuous measurement mode
        self.i2c.write_byte_data(self.address, self._REG_MODE, 0x00)
        
    def read_scaled(self) -> tuple[float, float, float]:
        data = self.i2c.read_i2c_block_data(self.address, self._REG_DATA_OUT, 6)
        
        # Big Endian, order is X, Z, Y
        x = (data[0] << 8) | data[1]
        z = (data[2] << 8) | data[3]
        y = (data[4] << 8) | data[5]
        
        x = x - 65536 if x >= 32768 else x
        y = y - 65536 if y >= 32768 else y
        z = z - 65536 if z >= 32768 else z
        
        # HMC5883L typical scale factor ~ 0.92 mG/LSB for 1.3 Ga
        scale = 0.92
        return x * scale, y * scale, z * scale


@SensorRegistry.register
class NativeSEN0140(IMUBase):
    """
    Sterownik dla modułu DFRobot SEN0140 (10-DoF).
    Driver for DFRobot SEN0140 module (10-DoF).
    """

    DRIVER_NAME = "native_sen0140"
    # Adresy inicjalizacyjne to akcelerometr i żyroskop
    I2C_ADDRESSES = [0x53, 0x68]
    PRIORITY = 25

    @classmethod
    def scan(cls, i2c) -> bool:
        """Sprawdza obecność ADXL345 (0x53) i ITG3205 (0x68)."""
        try:
            # ADXL345 check
            dev_id = i2c.read_byte_data(0x53, 0x00)
            if dev_id != 0xE5:
                return False
            # ITG3205 check
            i2c.read_byte_data(0x68, 0x00)
            return True
        except (OSError, IOError):
            return False

    def __init__(self, i2c_wrapper: I2CWrapper) -> None:
        self.i2c: I2CWrapper = i2c_wrapper
        self.accel = None
        self.gyro = None
        self.mag = None
        self.baro = None
        
        # ADXL345 (Accel)
        try:
            self.accel = NativeADXL345(self.i2c, address=0x53)
            logger.info("✓ SEN0140: ADXL345 initialized at 0x53")
        except Exception as e:
            logger.error(f"✗ SEN0140: ADXL345 init failed: {e}")
            
        # ITG3205 (Gyro)
        try:
            self.gyro = NativeITG3205(self.i2c, address=0x68)
            logger.info("✓ SEN0140: ITG3205 initialized at 0x68")
        except Exception as e:
            logger.error(f"✗ SEN0140: ITG3205 init failed: {e}")
            
        # Magnetometer: VCM5883L/HMC5883L (0x1E) or QMC5883L (0x0D)
        try:
            self.mag = NativeHMC5883L(self.i2c, address=0x1E)
            logger.info("✓ SEN0140: HMC5883L/VCM5883L initialized at 0x1E")
        except Exception:
            try:
                self.mag = NativeQMC5883L(self.i2c, address=0x0D)
                logger.info("✓ SEN0140: QMC5883L initialized at 0x0D")
            except Exception as e2:
                logger.error(f"✗ SEN0140: Magnetometer init failed: {e2}")

        # Barometer: BMP280 or BMP180
        # W zależności od wersji płytki (BMP085/180 na 0x77, BMP280 na 0x76/0x77)
        for bmp_class in [NativeBMP280, NativeBMP180]:
            for addr in [0x76, 0x77]:
                try:
                    self.baro = bmp_class(self.i2c, address=addr)
                    logger.info(f"✓ SEN0140: {bmp_class.__name__} initialized at {hex(addr)}")
                    break
                except Exception:
                    continue
            if self.baro:
                break
        
        if not self.baro:
            logger.warning("✗ SEN0140: Barometer init failed")

    def read_data(self) -> dict[str, Any]:
        """
        Zwraca dane ze wszystkich dostępnych czujników modułu.
        Returns data from all available sensors of the module.
        """
        data: dict[str, Any] = {}
        
        if self.accel:
            try:
                ax, ay, az = self.accel.read_scaled()
                data.update({"ax": ax, "ay": ay, "az": az})
            except Exception:
                pass
                
        if self.gyro:
            try:
                gx, gy, gz = self.gyro.read_scaled()
                data.update({"gx": gx, "gy": gy, "gz": gz})
            except Exception:
                pass
                
        if self.mag:
            try:
                # Obejście dla różnych interfejsów (QMC zwraca tuple lub dict)
                if hasattr(self.mag, 'read_scaled'):
                    mx, my, mz = self.mag.read_scaled()
                    data.update({"mx": mx, "my": my, "mz": mz})
                elif hasattr(self.mag, 'read_data'):
                    mag_data = self.mag.read_data()
                    data.update(mag_data)
            except Exception:
                pass
                
        if self.baro:
            try:
                baro_data = self.baro.read_data()
                data.update(baro_data) # Dodaje 'temperature' i 'pressure'
                # Zmieniamy 'temperature' na 'temp' w standardzie RCSIM
                if 'temperature' in data:
                    data['temp'] = data.pop('temperature')
            except Exception:
                pass
                
        return data

    def calibrate(self) -> bool:
        """
        Brak natywnej procedury kalibracji sprzętowej w jednym układzie (jak MPU).
        Returns True if sensors are functional.
        """
        logger.info("SEN0140: Software calibration routines apply here.")
        return bool(self.accel and self.gyro)
