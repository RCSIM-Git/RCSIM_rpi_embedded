"""PL: Obsługuje wyjścia PWM układu PCA9685 w osobnym, taktowanym wątku.

Wątek utrzymuje cykliczne wysyłanie sterowania niezależnie od obciążenia SLAM/AI.

EN: Handles PCA9685 PWM outputs in a separate, periodically scheduled thread.

The thread maintains regular control updates independently of SLAM/AI load.

Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""

import logging
import threading
import time
from typing import Dict
class ActuatorWorker(threading.Thread):
    """PL: Wątek wysyłający sterowanie do wyjść PWM układu PCA9685.

    EN: Thread that sends control commands to the PCA9685 PWM outputs.
    """

    def __init__(self, hw_manager: "HardwareManager", freq: int = 50) -> None:
        """PL: Inicjalizuje wątek i bufor ostatnich poleceń sterujących.

        Args:
            hw_manager: Menedżer sprzętu używany do zapisu wyjść.
            freq: Częstotliwość wysyłania sterowania w hercach.

        EN: Initializes the thread and the buffer for the latest control commands.

        Args:
            hw_manager: Hardware manager used to write the outputs.
            freq: Control output frequency in hertz.
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
        self.last_update_time = time.time()

    def set_commands(
        self, steering: float, throttle: float, extra: Dict[int, int], armed: bool
    ) -> None:
        """PL: Zastępuje bufor najnowszymi poleceniami sterowania.

        Args:
            steering: Wartość sterowania skrętem.
            throttle: Wartość sterowania przepustnicą.
            extra: Wartości dodatkowych kanałów PWM.
            armed: Informacja, czy wolno wysyłać sterowanie.

        EN: Replaces the buffer with the latest control commands.

        Args:
            steering: Steering control value.
            throttle: Throttle control value.
            extra: Values for additional PWM channels.
            armed: Whether control output is enabled.
        """
        with self.lock:
            self.steering = steering
            self.throttle = throttle
            self.extra_channels = extra.copy()
            self.armed = armed
            self.last_update_time = time.time()

    def run(self) -> None:
        """PL: Wysyła uzbrojone sterowanie w pętli o zadanej częstotliwości.

        Przy braku aktualizacji poleceń wysyła neutralne wartości dla
        bezpieczeństwa.

        EN: Sends armed control commands in a loop at the configured frequency.

        If command updates become stale, sends neutral values for safety.
        """
        self.running = True
        logging.info(f"ActuatorWorker started at {1.0/self.interval} Hz")

        next_tick = time.perf_counter_ns()
        interval_ns = int(self.interval * 1_000_000_000)

        while self.running:
            try:
                with self.lock:
                    s, t, extra, armed, last_update = (
                        self.steering,
                        self.throttle,
                        self.extra_channels,
                        self.armed,
                        self.last_update_time,
                    )

                if armed:
                    if time.time() - last_update > 0.2:
                        # Starvation check (Main thread hung)
                        logging.warning("ActuatorWorker: Main thread update timeout! Emergency neutralization.")
                        self.hw.write_controls(0.0, 0.0, {})
                    else:
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
        """PL: Ustawia flagę kończącą pętlę roboczą wątku.

        EN: Sets the flag that terminates the worker thread loop.
        """
        self.running = False
