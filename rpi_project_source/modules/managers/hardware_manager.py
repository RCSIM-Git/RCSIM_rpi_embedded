"""
Menedżer Sprzętu - Warstwa Abstrakcji Sprzętowej (HAL).
Hardware Manager - Hardware Abstraction Layer (HAL).

Zrefaktoryzowana Powłoka-Fasada API Głównego Menedżera sprzętu RPi.
Deleguje połączenia logiki do osobnych pakietów (SRP).
"""

import logging
from typing import Any

# Próba importu bibliotek sprzętowych (I2C layer initialization must reside closely)
try:
    from modules.drivers.native.native_i2c import I2CWrapper
except (ImportError, NotImplementedError):
    I2CWrapper = None

from modules.hardware.actuator_controller import ActuatorController
# Delegacja do zrefaktoryzowanych jednostek wykonawczych (SRP)
from modules.hardware.sensor_aggregator import SensorAggregator
from modules.utils.kinematics import estimate_motion

logger = logging.getLogger(__name__)


class HardwareManager:
    """
    Zrefaktoryzowana Fasada API zarządzająca RPi cyklem życia sprzętu
    Manages the entire hardware lifecycle acting as a transparent proxy.
    """

    def __init__(
        self, hardware_config: dict[str, Any], init_pca_neutral: bool = False
    ) -> None:
        self.config: dict[str, Any] = hardware_config
        self.init_pca_neutral = init_pca_neutral
        self.i2c = self._init_i2c()

        # Pakiety podłączone we wzorcu fasady
        self.sensors = SensorAggregator(self.config, self.i2c)
        self.actuators = ActuatorController(self.config, self.i2c, init_pca_neutral)

    def _init_i2c(self) -> Any | None:
        """Fizyczne otwarcie portu szyny I2C (Bus 1) dla warstwy komunikacji RPi."""
        if I2CWrapper is None:
            logger.warning(
                "Native I2C/smbus2 drivers not found. Running in mock hardware mode."
            )
            return None

        try:
            i2c_bus = I2CWrapper(bus_num=1)
            logger.info("I2C bus initialized successfully on bus 1.")
            return i2c_bus
        except Exception as e:
            logger.critical(
                f"Failed to initialize I2C: {e}. "
                "PLEASE CHECK HARDWARE CONNECTIONS "
                "AND I2C BUS (i2cdetect -y 1). "
                "I2C hardware will be disabled."
            )
            return None

    # ==============================================================
    # Właściwości udostępniające (Backward Compatibility Proxy)
    # ==============================================================

    @property
    def lidar(self) -> Any:
        return self.sensors.lidar

    @property
    def gps(self) -> Any:
        return self.sensors.gps

    @property
    def imu(self) -> Any:
        return self.sensors.imu

    @property
    def ups(self) -> Any:
        return self.sensors.ups

    @property
    def pca(self) -> Any:
        return self.actuators.pca

    # ==============================================================
    # Metody delektujące (API Compatibility) / Proxy Methods API
    # ==============================================================

    def read_sensors(self) -> dict[str, Any]:
        """Polowanie odczytów skondensowane w Aggregatorze."""
        return self.sensors.read_sensors()

    def set_steering(self, value: float) -> None:
        """Popycha pozycję układu kierowniczego za pomocą mapowania krzywych PWM wewnątrz."""
        self.actuators.set_steering(value)

    def set_throttle(self, value: float) -> None:
        """Zapisanie wychylenia gazu (dla korekt) oraz transfer logiczny sygnału ESC."""
        self.sensors.notify_throttle_change(value)
        self.actuators.set_throttle(value)

    def set_channel_pulse(self, channel: int, pulse_us: int) -> None:
        self.actuators.set_channel_pulse(channel, pulse_us)

    def write_controls(
        self, steering: float, throttle: float, extra: dict[int, int]
    ) -> None:
        self.set_steering(steering)
        self.set_throttle(throttle)
        for ch, pulse_us in extra.items():
            self.set_channel_pulse(ch, pulse_us)

    def set_pwm(self, pulse_us: int, force: bool = False, channel: int = -1) -> None:
        self.actuators.set_pwm(pulse_us, force, channel)

    def calibrate_esc(self, channel: int = -1) -> None:
        self.actuators.calibrate_esc(channel)

    def disable_all_channels(self) -> None:
        self.actuators.disable_all_channels()

    def arm_pca(self) -> None:
        self.actuators.arm_pca()

    def calibrate_imu(self) -> bool:
        return self.sensors.calibrate_imu()

    def estimate_motion(
        self,
        throttle: float,
        steering: float,
        dt: float,
        imu_data: dict[str, Any] | None = None,
    ) -> tuple[float, float, float]:
        """Statyczny bypass do biblioteki matematyki w utilsach oddzielonej od I2C."""
        max_s = self.config.get("max_speed_mps", 2.0)
        max_str = self.config.get("max_steer_rad", 0.52)
        wb = self.config.get("wheelbase_m", 0.25)

        return estimate_motion(
            throttle=throttle,
            steering=steering,
            dt=dt,
            max_speed_mps=max_s,
            max_steer_rad=max_str,
            wheelbase_m=wb,
            imu_data=imu_data,
        )

    def cleanup(self) -> None:
        """Sekwencyjne zwalnianie przerw i zasilania UART i I2C."""
        logger.info("HardwareManager / Facade: Cleanup...")
        self.actuators.cleanup()
        self.sensors.cleanup()
