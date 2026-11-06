"""PL: Udostępnia fasadę do zarządzania czujnikami i elementami wykonawczymi.
EN: Provides a facade for managing sensors and actuators.
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
from modules.utils.kinematics import estimate_motion, ControlInput, KinematicConfig

logger = logging.getLogger(__name__)


class HardwareManager:
    """PL: Łączy inicjalizację I2C, agregację czujników i sterowanie PWM.
    EN: Combines I2C initialization, sensor aggregation, and PWM control.
    """

    def __init__(
        self, hardware_config: dict[str, Any], init_pca_neutral: bool = False
    ) -> None:
        """PL: Inicjalizuje magistralę I2C i podmoduły sprzętowe.
        EN: Initializes the I2C bus and hardware submodules.
        """
        self.config: dict[str, Any] = hardware_config
        self.init_pca_neutral = init_pca_neutral
        self.i2c = self._init_i2c()

        # Pakiety podłączone we wzorcu fasady
        self.sensors = SensorAggregator(self.config, self.i2c)
        self.actuators = ActuatorController(self.config, self.i2c, init_pca_neutral)

    def _init_i2c(self) -> Any | None:
        """PL: Otwiera magistralę I2C numer 1, jeśli sterownik jest dostępny.
        EN: Opens I2C bus 1 when its driver is available.
        """
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
        """PL: Zwraca sterownik LiDAR z agregatora czujników.
        EN: Returns the LiDAR driver from the sensor aggregator.
        """
        return self.sensors.lidar

    @property
    def gps(self) -> Any:
        """PL: Zwraca odbiornik GPS z agregatora czujników.
        EN: Returns the GPS receiver from the sensor aggregator.
        """
        return self.sensors.gps

    @property
    def imu(self) -> Any:
        """PL: Zwraca czujnik IMU z agregatora czujników.
        EN: Returns the IMU sensor from the sensor aggregator.
        """
        return self.sensors.imu

    @property
    def ups(self) -> Any:
        """PL: Zwraca monitor zasilania UPS z agregatora czujników.
        EN: Returns the UPS power monitor from the sensor aggregator.
        """
        return self.sensors.ups

    @property
    def pca(self) -> Any:
        """PL: Zwraca kontroler PWM PCA9685.
        EN: Returns the PCA9685 PWM controller.
        """
        return self.actuators.pca

    # ==============================================================
    # Metody delektujące (API Compatibility) / Proxy Methods API
    # ==============================================================

    def read_sensors(self) -> dict[str, Any]:
        """PL: Zwraca zagregowane odczyty czujników.
        EN: Returns aggregated sensor readings.
        """
        return self.sensors.read_sensors()

    def set_steering(self, value: float) -> None:
        """PL: Przekazuje wychylenie kierownicy do sterownika PWM.
        EN: Sends the steering command to the PWM actuator controller.
        """
        self.actuators.set_steering(value)

    def set_throttle(self, value: float) -> None:
        """PL: Zapisuje gaz dla kompensacji IMU i ustawia wyjście ESC.
        EN: Stores throttle for IMU compensation and sets the ESC output.
        """
        self.sensors.notify_throttle_change(value)
        self.actuators.set_throttle(value)

    def set_channel_pulse(self, channel: int, pulse_us: int) -> None:
        """PL: Ustawia impuls PWM na wskazanym kanale.
        EN: Sets a PWM pulse on the selected channel.
        """
        self.actuators.set_channel_pulse(channel, pulse_us)

    def write_controls(
        self, steering: float, throttle: float, extra: dict[int, int]
    ) -> None:
        """PL: Zapisuje sterowanie kierownicą, gazem i kanałami dodatkowymi.
        EN: Writes steering, throttle, and additional channel controls.
        """
        self.set_steering(steering)
        self.set_throttle(throttle)
        for ch, pulse_us in extra.items():
            self.set_channel_pulse(ch, pulse_us)

    def set_pwm(self, pulse_us: int, force: bool = False, channel: int = -1) -> None:
        """PL: Przekazuje bezpośrednie ustawienie impulsu PWM.
        EN: Forwards a direct PWM pulse command.
        """
        self.actuators.set_pwm(pulse_us, force, channel)

    def calibrate_esc(self, channel: int = -1) -> None:
        """PL: Uruchamia kalibrację wskazanego regulatora ESC.
        EN: Starts calibration for the selected ESC.
        """
        self.actuators.calibrate_esc(channel)

    def disable_all_channels(self) -> None:
        """PL: Wyłącza wszystkie wyjścia kontrolera PCA9685.
        EN: Disables all PCA9685 controller outputs.
        """
        self.actuators.disable_all_channels()

    def arm_pca(self) -> None:
        """PL: Ustawia neutralne sygnały i uzbraja kontroler PCA.
        EN: Sets neutral signals and arms the PCA controller.
        """
        self.actuators.arm_pca()

    def calibrate_imu(self) -> bool:
        """PL: Rozpoczyna kalibrację IMU i zwraca jej wynik.
        EN: Starts IMU calibration and returns its result.
        """
        return self.sensors.calibrate_imu()

    def estimate_motion(
        self,
        throttle: float,
        steering: float,
        dt: float,
        imu_data: dict[str, Any] | None = None,
    ) -> tuple[float, float, float]:
        """PL: Szacuje ruch pojazdu z wejść sterujących i odczytów IMU.
        EN: Estimates vehicle motion from control inputs and IMU readings.
        """
        max_s = self.config.get("max_speed_mps", 2.0)
        max_str = self.config.get("max_steer_rad", 0.52)
        wb = self.config.get("wheelbase_m", 0.25)

        control = ControlInput(throttle=throttle, steering=steering)
        config = KinematicConfig(max_speed_mps=max_s, max_steer_rad=max_str, wheelbase_m=wb)

        return estimate_motion(
            control=control,
            dt=dt,
            config=config,
            imu_data=imu_data,
        )

    def cleanup(self) -> None:
        """PL: Zamyka elementy wykonawcze i czujniki sprzętowe.
        EN: Shuts down hardware actuators and sensors.
        """
        logger.info("HardwareManager / Facade: Cleanup...")
        self.actuators.cleanup()
        self.sensors.cleanup()
