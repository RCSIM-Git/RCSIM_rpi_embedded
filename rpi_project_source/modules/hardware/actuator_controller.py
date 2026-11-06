"""PL: Steruje serwami i sygnałami PWM na Raspberry Pi.
EN: Controls servos and PWM signals on Raspberry Pi.

PL: Przelicza sterowanie pojazdem na impulsy dla kontrolera PCA9685.
EN: Converts vehicle controls into pulses for the PCA9685 controller.
"""

import logging
import time
from typing import Any

from modules.pca9685 import PCA9685

logger = logging.getLogger(__name__)


class EMAFilter:
    """PL: Wygładza wartości sygnału PWM filtrem wykładniczym.
    EN: Smooths PWM values with an exponential moving average.
    """

    def __init__(self, alpha: float):
        """PL: Tworzy filtr z ograniczonym współczynnikiem wygładzania.
        EN: Creates a filter with a bounded smoothing coefficient.
        """
        self.alpha = max(0.01, min(1.0, alpha))
        self.last_value: float | None = None

    def apply(self, value: float) -> float:
        """PL: Filtruje próbkę i zwraca wygładzoną wartość.
        EN: Filters a sample and returns the smoothed value.
        """
        if self.last_value is None:
            self.last_value = value
            return value
        self.last_value = (self.alpha * value) + ((1.0 - self.alpha) * self.last_value)
        return self.last_value


class SlewRateLimiter:
    """PL: Ogranicza tempo zmian wartości sygnału PWM.
    EN: Limits the rate of change of a PWM signal.
    """

    def __init__(self, max_rate_us_per_ms: float):
        """PL: Ustawia maksymalną szybkość zmian w mikrosekundach na milisekundę.
        EN: Sets the maximum change rate in microseconds per millisecond.
        """
        # Konwersja us/ms na us/s dla wewnętrznych obliczeń / Convert us/ms to us/s
        self.max_rate = max_rate_us_per_ms * 1000.0
        self.last_value: float | None = None
        self.last_time: float = 0.0

    def apply(self, target_value: float) -> int:
        """PL: Zwraca wartość docelową ograniczoną szybkością zmian.
        EN: Returns the target value constrained by the configured rate.
        """
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
    """PL: Mapuje polecenia sterujące na sygnały sprzętowych kanałów PWM.
    EN: Maps control commands to hardware PWM channel signals.
    """

    def __init__(
        self,
        hardware_config: dict[str, Any],
        i2c_bus: Any,
        init_pca_neutral: bool = False,
    ):
        """PL: Konfiguruje kanały, filtry i kontroler PCA9685.
        EN: Configures channels, filters, and the PCA9685 controller.
        """
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
        """PL: Inicjalizuje PCA9685 z ustawieniami sprzętowymi i failsafe.
        EN: Initializes the PCA9685 with hardware and failsafe settings.
        """
        pca_config = self.config.get("pca9685", {})
        freq = pca_config.get("frequency", 50)

        # V39.12: Dynamiczny timeout watchdoga zsynchronizowany z GCS
        wd_ms = self.config.get("watchdog_timeout_ms", 500)
        failsafe_timeout_s = wd_ms / 1000.0

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
                failsafe_timeout=failsafe_timeout_s,
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
        """PL: Przelicza i wysyła wychylenie kierownicy jako impuls PWM.
        EN: Converts and sends the steering command as a PWM pulse.
        """
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
        """PL: Przelicza i wysyła gaz jako impuls PWM.
        EN: Converts and sends throttle as a PWM pulse.
        """
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
        """PL: Ogranicza i filtruje impuls przed wysłaniem go do kanału.
        EN: Clamps and filters a pulse before sending it to the channel.
        """
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
        """PL: Zapisuje sterowanie kierownicą, gazem i dodatkowymi kanałami.
        EN: Writes steering, throttle, and additional channel commands.
        """
        self.set_steering(steering)
        self.set_throttle(throttle)
        for ch, pulse_us in extra.items():
            self.set_channel_pulse(ch, pulse_us)

    def set_pwm(self, pulse_us: int, force: bool = False, channel: int = -1) -> None:
        """PL: Ustawia bezpośrednio impuls PWM na wskazanym kanale.
        EN: Directly sets a PWM pulse on the selected channel.
        """
        target_channel = self.throttle_channel if channel == -1 else channel
        if self.pca:
            self.pca.set_servo_pulse(target_channel, pulse_us, force=force)

    def calibrate_esc(self, channel: int = -1) -> None:
        """PL: Uruchamia procedurę kalibracji regulatora ESC.
        EN: Starts the ESC calibration procedure.
        """
        target_channel = self.throttle_channel if channel == -1 else channel
        if self.pca:
            self.pca.calibrate_esc(target_channel)

    def disable_all_channels(self) -> None:
        """PL: Wyłącza wszystkie kanały kontrolera PWM.
        EN: Disables all PWM controller channels.
        """
        if self.pca:
            self.pca.disable_all_channels()

    def arm_pca(self) -> None:
        """PL: Ustawia neutralne sterowanie i uzbraja kontroler PCA.
        EN: Sets neutral controls and arms the PCA controller.
        """
        if self.pca:
            self.set_steering(0.0)
            self.set_throttle(0.0)
            logger.info(
                "ActuatorController: PCA Armed. "
                f"Steer Ch{self.steering_channel}, Thr Ch{self.throttle_channel} to Neutral."
            )

    def cleanup(self) -> None:
        """PL: Ustawia sygnały neutralne i zamyka kontroler w trybie failsafe.
        EN: Sets neutral signals and closes the controller in failsafe mode.
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
