"""
Actuator Controller - Menedżer Serwomechanizmów i Sygnałów PWM (RPi).
Actuator Controller - Servomechanisms and PWM signals manager (RPi).

Zajmuje się izolacją logiki przeliczania wychyleń kierownicy [-1, 1] i przepustnicy na mikrosekundy używane
w kontrolerach sprzętowych PCA9685/Hatak.
"""

import logging
import time
from typing import Any

from modules.pca9685 import PCA9685

logger = logging.getLogger(__name__)


class EMAFilter:
    """Filtr wygładzający sygnał PWM. / PWM signal smoothing filter."""

    def __init__(self, alpha: float):
        self.alpha = max(0.01, min(1.0, alpha))
        self.last_value: float | None = None

    def apply(self, value: float) -> float:
        if self.last_value is None:
            self.last_value = value
            return value
        self.last_value = (self.alpha * value) + ((1.0 - self.alpha) * self.last_value)
        return self.last_value


class SlewRateLimiter:
    """
    Ogranicznik prędkości narastania sygnału. / Slew Rate Limiter.
    Zabezpiecza sprzęt przed gwałtownymi skokami PWM.
    """

    def __init__(self, max_rate_us_per_ms: float):
        # Konwersja us/ms na us/s dla wewnętrznych obliczeń / Convert us/ms to us/s
        self.max_rate = max_rate_us_per_ms * 1000.0
        self.last_value: float | None = None
        self.last_time: float = 0.0

    def apply(self, target_value: float) -> int:
        now = time.time()
        if self.last_value is None:
            self.last_value = target_value
            self.last_time = now
            return int(target_value)

        dt = now - self.last_time
        if dt <= 0:
            return int(self.last_value)

        # Oblicz maksymalną dozwoloną zmianę / Calculate max allowed change
        max_change = self.max_rate * dt
        delta = target_value - self.last_value

        if abs(delta) > max_change:
            delta = max_change if delta > 0 else -max_change

        self.last_value += delta
        self.last_time = now
        return int(self.last_value)


class ActuatorController:
    """
    Sub-moduł (SRP) odseparowany od dawnego God-Objectu `HardwareManager`.
    Obejmuje wyłączne operacje matematyczne rzutowania sygnałów PWM i komunikację wysyłania komend do osi pojazdu.
    """

    def __init__(
        self,
        hardware_config: dict[str, Any],
        i2c_bus: Any,
        init_pca_neutral: bool = False,
    ):
        self.config = hardware_config
        self.i2c = i2c_bus
        self.pca: PCA9685 | None = None
        self.init_pca_neutral = init_pca_neutral

        self.steering_channel: int = hardware_config.get("steering_channel", 0)
        self.throttle_channel: int = hardware_config.get("throttle_channel", 1)

        self.steering_range: tuple[int, int] = (
            hardware_config.get("steering_min", 1000),
            hardware_config.get("steering_max", 2000),
        )
        self.throttle_range: tuple[int, int] = (
            hardware_config.get("throttle_min", 1000),
            hardware_config.get("throttle_max", 2000),
        )

        self.steering_neutral: int = hardware_config.get(
            "steering_neutral", (self.steering_range[0] + self.steering_range[1]) // 2
        )
        self.throttle_neutral: int = hardware_config.get(
            "throttle_neutral", (self.throttle_range[0] + self.throttle_range[1]) // 2
        )

        self.steering_inverted: bool = hardware_config.get("steering_inverted", False)
        self.throttle_inverted: bool = hardware_config.get("throttle_inverted", False)

        # --- [SAFETY-008] Slew Rate & EMA Tuning ---
        # Slew Rate: default 15 us/ms (Standard). Ultra fast digital can go 50+.
        slew_rate = hardware_config.get("slew_rate_limit", 15.0)

        # EMA Alpha: 1.0 = Off (Standard). 0.1-0.5 = Smoothing (High Jitter AI).
        ema_alpha = hardware_config.get("pwm_ema_alpha", 1.0)

        self.limiters: dict[int, SlewRateLimiter] = {
            self.steering_channel: SlewRateLimiter(slew_rate),
            self.throttle_channel: SlewRateLimiter(slew_rate),
        }

        self.ema_filters: dict[int, EMAFilter] = {
            self.steering_channel: EMAFilter(ema_alpha),
            self.throttle_channel: EMAFilter(ema_alpha),
        }

        self._init_pca9685()

    def _init_pca9685(self) -> None:
        """Inicjalizuje kontroler PWM PCA9685 za pomocą ujednoliconego wrappera."""
        pca_config = self.config.get("pca9685", {})
        freq = pca_config.get("frequency", 50)

        try:
            self.pca = PCA9685(
                i2c_bus=self.i2c,
                logger=logger,
                frequency=freq,
                init_neutral=self.init_pca_neutral,
                oscillator_freq=pca_config.get("oscillator_freq", 25000000),
                auto_calibrate=pca_config.get("auto_calibrate", False),
                calib_ch=pca_config.get("calibration_channel", 15),
                calib_gpio=pca_config.get("calibration_gpio", 17),
                oe_pin=pca_config.get("oe_pin"),
            )

            if self.pca.pca:
                logger.info(
                    f"ActuatorController: PCA9685 ready via {self.pca.driver_type} driver."
                )
            else:
                logger.warning(
                    "ActuatorController: PCA9685 initialization failed (Mock Mode)."
                )

        except Exception as e:
            logger.error(f"ActuatorController: PCA9685 Critical Error: {e}")
            self.pca = None

    def set_steering(self, value: float) -> None:
        if not self.pca:
            return

        if self.steering_inverted:
            value = -value

        p_min, p_max = self.steering_range
        p_center = self.steering_neutral

        if value >= 0:
            pulse_us = int(p_center + (value * (p_max - p_center)))
        else:
            pulse_us = int(p_center + (value * (p_center - p_min)))

        self.set_channel_pulse(self.steering_channel, pulse_us)

    def set_throttle(self, value: float) -> None:
        if not self.pca:
            return

        if self.throttle_inverted:
            value = -value

        p_min, p_max = self.throttle_range
        p_center = self.throttle_neutral

        if value >= 0:
            pulse_us = int(p_center + (value * (p_max - p_center)))
        else:
            pulse_us = int(p_center + (value * (p_center - p_min)))

        self.set_channel_pulse(self.throttle_channel, pulse_us)

    def set_channel_pulse(self, channel: int, pulse_us: int) -> None:
        """Sets raw pulse with Hard Clamping [PLAN-003] and Slew Rate Limiting [SAFETY-008]."""
        if self.pca:
            # 1. Hard Clamping for safety
            if channel == self.steering_channel:
                pulse_us = max(
                    self.steering_range[0], min(self.steering_range[1], pulse_us)
                )
            elif channel == self.throttle_channel:
                pulse_us = max(
                    self.throttle_range[0], min(self.throttle_range[1], pulse_us)
                )
            else:
                pulse_us = max(800, min(2200, pulse_us))

            # 2. EMA Filtering
            ema = self.ema_filters.get(channel)
            if ema:
                pulse_us = ema.apply(pulse_us)

            # 3. Slew Rate Limiting
            limiter = self.limiters.get(channel)
            if limiter:
                pulse_us = limiter.apply(pulse_us)

            self.pca.set_servo_pulse(channel, int(pulse_us))

    def write_controls(
        self, steering: float, throttle: float, extra: dict[int, int]
    ) -> None:
        self.set_steering(steering)
        self.set_throttle(throttle)
        for ch, pulse_us in extra.items():
            self.set_channel_pulse(ch, pulse_us)

    def set_pwm(self, pulse_us: int, force: bool = False, channel: int = -1) -> None:
        target_channel = self.throttle_channel if channel == -1 else channel
        if self.pca:
            self.pca.set_servo_pulse(target_channel, pulse_us, force=force)

    def calibrate_esc(self, channel: int = -1) -> None:
        target_channel = self.throttle_channel if channel == -1 else channel
        if self.pca:
            self.pca.calibrate_esc(target_channel)

    def disable_all_channels(self) -> None:
        if self.pca:
            self.pca.disable_all_channels()

    def arm_pca(self) -> None:
        if self.pca:
            self.set_steering(0.0)
            self.set_throttle(0.0)
            logger.info(
                "ActuatorController: PCA Armed. "
                f"Steer Ch{self.steering_channel}, Thr Ch{self.throttle_channel} to Neutral."
            )

    def cleanup(self) -> None:
        """
        Gwarantuje bezpieczne zatrzymanie (Fail-Safe) [PLAN-003].
        Sets neutral signals before disabling PCA.
        """
        logger.info("ActuatorController: Fail-Safe Triggered. Setting neutrals...")
        try:
            # Set physical neutrals from config
            self.set_steering(0.0)
            self.set_throttle(0.0)
            # Short wait for ESCs to catch up before power-off
            import time

            time.sleep(0.05)
        except Exception as e:
            logger.error(f"ActuatorController: Fail-Safe error: {e}")

        if self.pca:
            self.pca.cleanup()
