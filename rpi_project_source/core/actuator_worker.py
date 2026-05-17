"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Wątek dedykowany do obsługi wyjść PWM (PCA9685).
Dedicated thread for handling PWM outputs (PCA9685).

Zapewnia stabilne taktowanie sygnału wyjściowego niezależnie od obciążeń SLAM/AI.
Ensures stable output timing regardless of SLAM/AI load.
"""

import logging
import threading
import time
from typing import TYPE_CHECKING, Dict

if TYPE_CHECKING:
    from modules.managers.hardware_manager import HardwareManager


class ActuatorWorker(threading.Thread):
    """
    Wątek dedykowany do obsługi wyjść PWM (PCA9685).
    Dedicated thread for handling PWM outputs (PCA9685).

    Zapewnia stabilne taktowanie sygnału wyjściowego niezależnie od obciążeń SLAM/AI.
    Ensures stable output timing regardless of SLAM/AI load.
    """

    def __init__(self, hw_manager: "HardwareManager", freq: int = 50) -> None:
        """
        Inicjalizuje wątek roboczy układu wykonawczego.
        Initializes the actuator worker thread.

        Args:
            hw_manager (HardwareManager): Menadżer sprzętu. / Hardware manager.
            freq (int): Częstotliwość sterowania (Hz). / Control frequency (Hz).
        """
        super().__init__(daemon=True)
        self.hw = hw_manager
        self.running = False
        self.interval = 1.0 / freq
        self.lock = threading.Lock()

        # Buffer for latest commands
        self.steering = 0.0
        self.throttle = 0.0
        self.extra_channels: Dict[int, int] = {}
        self.armed = False

    def set_commands(
        self, steering: float, throttle: float, extra: Dict[int, int], armed: bool
    ) -> None:
        """
        Zapisuje nowe komendy do bufora wątku.
        Writes new commands to the thread buffer.

        Args:
            steering (float): Skręt. / Steering.
            throttle (float): Przepustnica. / Throttle.
            extra (Dict[int, int]): Dodatkowe kanały. / Extra channels.
            armed (bool): Czy system uzbrojony. / Is system armed.
        """
        with self.lock:
            self.steering = steering
            self.throttle = throttle
            self.extra_channels = extra.copy()
            self.armed = armed

    def run(self) -> None:
        """
        Główna pętla wątku z precyzyjnym wymuszeniem taktowania.
        Main thread loop with precise timing enforcement.
        """
        self.running = True
        logging.info(f"ActuatorWorker started at {1.0/self.interval} Hz")

        next_tick = time.perf_counter_ns()
        interval_ns = int(self.interval * 1_000_000_000)

        while self.running:
            try:
                with self.lock:
                    s, t, extra, armed = (
                        self.steering,
                        self.throttle,
                        self.extra_channels,
                        self.armed,
                    )

                if armed:
                    self.hw.write_controls(s, t, extra)

                next_tick += interval_ns
                now = time.perf_counter_ns()
                if next_tick > now:
                    sleep_time_s = (next_tick - now) / 1_000_000_000
                    if sleep_time_s > 0.002:
                        time.sleep(sleep_time_s - 0.002)
                    while time.perf_counter_ns() < next_tick:
                        pass
                else:
                    next_tick = time.perf_counter_ns()
            except Exception as e:
                logging.error(f"ActuatorWorker error: {e}")
                time.sleep(0.02)

    def stop(self) -> None:
        """
        Zatrzymuje wątek.
        Stops the thread.
        """
        self.running = False
