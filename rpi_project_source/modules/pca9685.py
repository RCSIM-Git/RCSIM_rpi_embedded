#!/usr/bin/env python3
"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Moduł do obsługi 16-kanałowego kontrolera serw PWM PCA9685 na Raspberry Pi.
Zoptymalizowany pod kątem aut RC i trudnych w kalibracji ESC.
"""

import atexit
import logging
import sys
import time
from typing import Any

# Wykrywanie dostępnych bibliotek
try:
    import adafruit_pca9685
    import board
    import busio

    ADAFRUIT_AVAILABLE = True
except ImportError:
    ADAFRUIT_AVAILABLE = False
    board = busio = adafruit_pca9685 = None

try:
    from modules.drivers.native.native_i2c import I2CWrapper
    from modules.drivers.native.native_pca9685 import NativePCA9685

    NATIVE_AVAILABLE = True
except (ImportError, NotImplementedError, ModuleNotFoundError):
    NATIVE_AVAILABLE = False
    NativePCA9685 = I2CWrapper = None

try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

I2C_AVAILABLE = ADAFRUIT_AVAILABLE or NATIVE_AVAILABLE


class PCA9685:
    """
    Sterownik magistrali I2C PWM dla serwomechanizmów (PCA9685).
    I2C PWM servo controller driver (PCA9685).
    """

    PULSE_MIN = 1000  # Max Tył / Hamulec
    PULSE_NEUTRAL = 1500  # Luz (Środek)
    PULSE_MAX = 2000  # Max Przód

    # --- SAFETY [PLAN-011] ---
    HARD_LIMIT_MIN = 800  # Bezwzględne minimum (zapobiega uszkodzeniu serwa)
    HARD_LIMIT_MAX = 2200  # Bezwzględne maksimum
    FAILSAFE_TIMEOUT = 0.5  # Sekundy bez komendy -> Neutral

    def __init__(
        self,
        i2c_bus: Any = None,
        logger: logging.Logger | None = None,
        address: int = 0x40,
        frequency: int = 50,
        init_neutral: bool = True,
        oscillator_freq: int = 25000000,
        auto_calibrate: bool = False,
        calib_ch: int = 15,
        calib_gpio: int = 17,
        oe_pin: int | None = None,
    ) -> None:
        """
        Inicjalizuje układ PCA9685.
        Initializes the PCA9685 chip.
        """
        self.oscillator_freq = oscillator_freq
        self.auto_calibrate = auto_calibrate
        self.calib_ch = calib_ch
        self.calib_gpio = calib_gpio
        self.oe_pin = oe_pin
        self._last_error_time = 0.0
        self._error_msg_cooldown = 5.0
        self.logger = logger or logging.getLogger(__name__)
        self.pca = None
        self.driver_type = "None"
        self._frequency = frequency

        # --- SAFETY STATE [PLAN-011] ---
        self._last_command_time = 0.0
        self._failsafe_active = False
        self._failsafe_notified = False

        if not I2C_AVAILABLE:
            self.logger.warning("PCA9685: Brak sterowników I2C. Tryb symulacji.")
            return

        # --- GPIO SETUP (OE PIN) ---
        if GPIO_AVAILABLE and self.oe_pin is not None:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                # Ustawiamy OE na HIGH na starcie (wyjścia wyłączone)
                GPIO.setup(self.oe_pin, GPIO.OUT, initial=GPIO.HIGH)
                self.logger.info(f"PCA9685: OE Pin configured on GPIO {self.oe_pin} (Active-Low).")
            except Exception as e:
                self.logger.error(f"PCA9685: Failed to setup OE pin: {e}")

        # --- 1. Próba: NATIVE DRIVER ---
        if NATIVE_AVAILABLE:
            try:
                if i2c_bus is None or (
                    I2CWrapper and not isinstance(i2c_bus, I2CWrapper)
                ):
                    bus_to_use = I2CWrapper(bus_num=1)
                else:
                    bus_to_use = i2c_bus

                self.pca = NativePCA9685(
                    bus_to_use, address=address, reference_clock_speed=oscillator_freq
                )
                self.pca.set_frequency(frequency)
                self.driver_type = "Native"
                self.logger.info(f"PCA9685: Driver NATIVE OK (0x{address:02X}).")
            except Exception as e:
                self.logger.warning(
                    f"PCA9685: Driver NATIVE nieudany: {e}. Próba fallback..."
                )
                self.pca = None

        # --- 2. Próba: ADAFRUIT DRIVER ---
        if self.pca is None and ADAFRUIT_AVAILABLE:
            try:
                if i2c_bus is None or (busio and not isinstance(i2c_bus, busio.I2C)):
                    bus_to_use = busio.I2C(board.SCL, board.SDA)
                else:
                    bus_to_use = i2c_bus

                # POPRAWKA: Dodano reference_clock_speed, by oscylator działał prawidłowo!
                self.pca = adafruit_pca9685.PCA9685(
                    bus_to_use,
                    address=address,
                    reference_clock_speed=self.oscillator_freq,
                )
                self.pca.frequency = frequency
                self.driver_type = "Adafruit"
                self.logger.info(f"PCA9685: Driver ADAFRUIT OK (0x{address:02X}).")
            except Exception as e:
                self.logger.error(f"PCA9685: Driver ADAFRUIT nieudany: {e}")
                self.pca = None

        # --- Konfiguracja startowa ---
        if self.pca:
            atexit.register(self.disable_all_channels)

            # --- [AUTO-CALIBRATION] ---
            if self.auto_calibrate:
                self.autocalibrate_oscillator()

            if init_neutral:
                self.set_all_channels_neutral()
            else:
                self.logger.info("PCA9685: Silent Start (High-Z).")
                self.disable_all_channels()
        else:
            self.logger.critical("PCA9685: FATAL - Brak sterowników!")

    def set_servo_pulse(self, channel: int, pulse_us: int, force: bool = False) -> None:
        """
        Ustawia impuls serwa w mikrosekundach.
        Sets servo pulse in microseconds.
        """
        if self.pca is None or not (0 <= channel < 16):
            return

        # 1. Hard Clamping: Nigdy nie wychodź poza fizyczne granice
        pulse_us = max(self.HARD_LIMIT_MIN, min(self.HARD_LIMIT_MAX, pulse_us))

        # 2. Soft Clamping: Normalny zakres ruchu
        if not force:
            pulse_us = max(self.PULSE_MIN, min(self.PULSE_MAX, pulse_us))

        # 3. Heartbeat: Resetujemy timer failsafe
        self._last_command_time = time.time()
        self._failsafe_active = False
        
        # 4. OE: Aktywacja wyjść (jeśli były wyłączone)
        if GPIO_AVAILABLE and self.oe_pin is not None and pulse_us > 0:
            if GPIO.input(self.oe_pin) == GPIO.HIGH:
                GPIO.output(self.oe_pin, GPIO.LOW)
                self.logger.debug("PCA9685: OE LOW (Outputs Enabled).")

        try:
            if self.driver_type == "Native":
                if hasattr(self.pca, "set_us"):
                    self.pca.set_us(channel, int(pulse_us))
                else:
                    period_us = 1_000_000.0 / self._frequency
                    off = int((pulse_us / period_us) * 4096)
                    self.pca.set_pwm(channel, 0, off)

            elif self.driver_type == "Adafruit":
                period_us = 1_000_000.0 / self._frequency
                duty = int((pulse_us / period_us) * 0xFFFF)
                duty = max(0, min(0xFFFF, duty))
                self.pca.channels[channel].duty_cycle = duty

        except Exception as e:
            now = time.time()
            if now - self._last_error_time > self._error_msg_cooldown:
                diag = self.diagnose_error(e)
                self.logger.error(f"PCA9685 Hardware Error: {diag}")
                self._last_error_time = now

    def diagnose_error(self, e: Exception) -> str:
        """
        Tłumaczy surowe błędy I2C na język 'ludzki'.
        Translates raw I2C errors into human-readable language.
        """
        err_msg = str(e)
        if "121" in err_msg or "Remote I/O error" in err_msg:
            return "Błąd I2C (Err 121): Brak komunikacji z PCA9685. Sprawdź kable SDA/SCL oraz czy kontroler ma zasilanie."
        if "16" in err_msg or "Device or resource busy" in err_msg:
            return "Błąd I2C (Err 16): Magistrala zajęta. Inny proces blokuje dostęp do sprzętu."
        if "OSError: [Errno 19]" in err_msg:
            return "Błąd I2C (Err 19): Urządzenie I2C odłączone w trakcie pracy."
        return f"Błąd sprzętowy: {err_msg} (Zalecany restart pojazdu)"

    def autocalibrate_oscillator(self) -> None:
        """
        Automatyczna kalibracja oscylatora PCA9685 (identycznie jak w ESP32).
        Wymaga fizycznej zworki między kanałem wyjściowym a pinem GPIO.
        """
        if not GPIO_AVAILABLE:
            self.logger.warning("Auto-Calibration: GPIO library not available. Skipping.")
            return

        if self.pca is None:
            return

        self.logger.info(
            f"Auto-Calibration: Starting... (PCA CH{self.calib_ch} -> GPIO {self.calib_gpio})"
        )

        try:
            # Ustawienie pinu GPIO jako wejście
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.calib_gpio, GPIO.IN)

            target_us = 1500
            attempts = 0
            max_attempts = 10
            error_threshold = 6  # 1 tick at 50Hz is ~4.88us, so 6us is the realistic limit

            # Start z domyślną częstotliwością
            current_osc_freq = self.oscillator_freq
            best_freq = current_osc_freq
            min_error = 999

            while attempts < max_attempts:
                # 1. Wyślij impuls 1500us (307 ticks dla 50Hz)
                self.set_servo_pulse(self.calib_ch, target_us, force=True)
                time.sleep(0.15)  # Zwiększony czas na stabilizację

                # 2. Zmierz rzeczywisty czas trwania impulsu (Uśredniony z 7 pomiarów)
                samples = []
                for _ in range(7):
                    m = self._measure_pulse_width(self.calib_gpio, timeout_s=0.05)
                    if m > 500:
                        samples.append(m)
                    time.sleep(0.01)

                if not samples:
                    self.logger.error(
                        f"Auto-Calibration: No valid signal on GPIO {self.calib_gpio}! "
                        "Check loopback cable."
                    )
                    break

                samples.sort()
                measured_us = samples[len(samples) // 2]
                error = abs(target_us - measured_us)

                # Śledzenie najlepszego wyniku
                if error < min_error:
                    min_error = error
                    best_freq = current_osc_freq

                self.logger.info(
                    f"Auto-Calibration Attempt {attempts+1}: Measured {measured_us:.1f}us | Error: {int(target_us - measured_us)}us"
                )

                if error <= error_threshold:
                    drift_hz = current_osc_freq - 25000000
                    drift_pct = (drift_hz / 25000000) * 100
                    self.logger.info(
                        f"Auto-Calibration SUCCESS! Final Freq: {current_osc_freq} Hz "
                        f"(Drift: {drift_hz:+} Hz / {drift_pct:+.2f}%)"
                    )
                    self.oscillator_freq = current_osc_freq
                    break

                # 3. Korekta sprzężenia zwrotnego z mocnym tłumieniem i limitami bezpieczeństwa
                correction = target_us / measured_us
                
                # Limit zmiany na krok (max +/- 30% na iterację), aby zapobiec eksplozji przy błędnym odczycie
                correction = max(0.7, min(1.3, correction))
                
                new_freq_target = int(current_osc_freq * correction)
                current_osc_freq = int(current_osc_freq * 0.4 + new_freq_target * 0.6)

                # 4. Zastosuj nową częstotliwość (Ostateczny limit bezpieczeństwa 15-35 MHz)
                current_osc_freq = max(15000000, min(35000000, current_osc_freq))
                if self.driver_type == "Native":
                    self.pca.reference_clock_speed = current_osc_freq
                    self.pca.set_frequency(self._frequency)
                elif self.driver_type == "Adafruit":
                    self.pca.frequency = self._frequency

                attempts += 1

            if attempts >= max_attempts:
                drift_hz = best_freq - 25000000
                drift_pct = (drift_hz / 25000000) * 100
                self.logger.warning(
                    f"Auto-Calibration: Max attempts reached. Best freq: {best_freq} Hz "
                    f"(Drift: {drift_hz:+} Hz / {drift_pct:+.2f}%, Error: {min_error:.1f}us)"
                )
                self.oscillator_freq = best_freq
                # Zastosuj najlepszą znalezioną
                if self.driver_type == "Native":
                    self.pca.reference_clock_speed = self.oscillator_freq
                    self.pca.set_frequency(self._frequency)

        except Exception as e:
            self.logger.error(f"Auto-Calibration: Unexpected error: {e}")
        finally:
            # Nie czyścimy GPIO całkowicie, aby nie psuć innych modułów,
            # ale zdejmujemy sygnał z kanału kalibracyjnego
            self.set_servo_pulse(self.calib_ch, 0, force=True)

    def _measure_pulse_width(self, gpio_pin: int, timeout_s: float = 0.1) -> float:
        """
        Mierzy czas trwania stanu wysokiego na pinie GPIO za pomocą przerwań.
        Użycie wait_for_edge jest bardziej stabilne niż pętla while.
        """
        try:
            # 1. Czekaj na zbocze narastające (początek impulsu)
            # Jeśli pin jest już HIGH, musimy poczekać aż spadnie i wzrośnie, 
            # ale dla 50Hz (20ms) timeout 0.1s jest bezpieczny.
            if GPIO.input(gpio_pin) == GPIO.HIGH:
                GPIO.wait_for_edge(gpio_pin, GPIO.FALLING, timeout=int(timeout_s * 1000))

            if GPIO.wait_for_edge(gpio_pin, GPIO.RISING, timeout=int(timeout_s * 1000)):
                pulse_start = time.perf_counter()
                # 2. Czekaj na zbocze opadające (koniec impulsu)
                if GPIO.wait_for_edge(gpio_pin, GPIO.FALLING, timeout=int(timeout_s * 1000)):
                    pulse_end = time.perf_counter()
                    return (pulse_end - pulse_start) * 1_000_000
        except Exception:
            pass
        return 0.0

    def check_failsafe(self) -> bool:
        """
        Sprawdza czy nie upłynął timeout od ostatniej komendy.
        Jeśli tak, odcina sygnał PWM na wszystkich kanałach (High-Z).
        Checks if timeout has passed since last command.
        If so, disables PWM signal on all channels (High-Z).
        """
        if self._last_command_time == 0.0:
            # Nie monitoruj, jeśli nigdy nie odebrano pierwszej komendy (Silent Start)
            return False

        now = time.time()
        if (now - self._last_command_time) > self.FAILSAFE_TIMEOUT:
            if not self._failsafe_active:
                if not self._failsafe_notified:
                    self.logger.warning(
                        f"PCA9685: FAILSAFE! Brak komend przez {self.FAILSAFE_TIMEOUT}s. "
                        "Odcięcie sygnału PWM (High-Z)."
                    )
                    self._failsafe_notified = True
                self.disable_all_channels()
                # OE: Fizyczne odcięcie
                if GPIO_AVAILABLE and self.oe_pin is not None:
                    GPIO.output(self.oe_pin, GPIO.HIGH)
                    self.logger.warning("PCA9685: OE HIGH (Outputs Disabled - Failsafe).")
                self._failsafe_active = True
            return True
        else:
            self._failsafe_active = False
            self._failsafe_notified = False

        return False

    def set_all_channels_neutral(self) -> None:
        """Ustawia neutralny impuls dla wszystkich kanałów. / Sets neutral pulse for all channels."""
        if not self.pca:
            return
        for i in range(16):
            self.set_servo_pulse(i, self.PULSE_NEUTRAL, force=True)

    def disable_all_channels(self) -> None:
        """Wyłącza wszystkie kanały PWM. / Disables all PWM channels."""
        if self.pca is None:
            return
        try:
            if self.driver_type == "Native":
                for i in range(16):
                    # POPRAWKA: Bezpieczne wyłączanie dla Native
                    if hasattr(self.pca, "set_pwm"):
                        self.pca.set_pwm(i, 0, 0)
                    elif hasattr(self.pca, "set_us"):
                        self.pca.set_us(i, 0)
            elif self.driver_type == "Adafruit":
                for i in range(16):
                    self.pca.channels[i].duty_cycle = 0
        except Exception:
            pass

    def cleanup(self) -> None:
        """Zwalnia zasoby sterownika. / Cleans up driver resources."""
        self.disable_all_channels()
        # OE: Fizyczne odcięcie na koniec
        if GPIO_AVAILABLE and self.oe_pin is not None:
            try:
                GPIO.output(self.oe_pin, GPIO.HIGH)
                self.logger.info("PCA9685: OE HIGH (Outputs Disabled - Cleanup).")
            except Exception:
                pass
        
        if self.pca:
            try:
                if hasattr(self.pca, "deinit"):
                    self.pca.deinit()
                elif hasattr(self.pca, "close"):
                    self.pca.close()
            except Exception:
                pass


# ==============================================================================
# NARZĘDZIA DIAGNOSTYCZNE I KALIBRACYJNE (INTERAKTYWNE MENU)
# ==============================================================================


def menu_kalibracji(pca: PCA9685, channel: int) -> None:
    """
    Uruchamia interaktywne menu kalibracji ESC.
    Starts interactive ESC calibration menu.
    """
    print("\n--- KREATOR KALIBRACJI ESC ---")
    print("1. ESC Z PRZYCISKIEM 'SET' (Surpass Hobby, Rocket-RC)")
    print("2. ESC BEZ PRZYCISKU / Crawlery (AM32)")
    print("3. Powrót")
    wybor = input("Wybierz typ swojego ESC: ")

    if wybor == "1":
        print("\n--- KALIBRACJA Z PRZYCISKIEM SET ---")
        print("[KROK 1] Upewnij się, że ESC jest WYŁĄCZONE.")
        pca.set_servo_pulse(channel, pca.PULSE_NEUTRAL, force=True)
        print("-> PCA9685 nadaje sygnał NEUTRALNY (1500us).")
        input(
            "Naciśnij i trzymaj przycisk SET na ESC, włącz zasilanie. Gdy dioda mruga, puść SET i wciśnij [ENTER]..."
        )

        input(
            "[KROK 2] Jesteś w trybie kalibracji. Naciśnij przycisk SET raz. Dioda mignie 1 raz. Potem wciśnij [ENTER]..."
        )

        pca.set_servo_pulse(channel, pca.PULSE_MAX, force=True)
        print("-> PCA9685 nadaje sygnał MAX PRZÓD (2000us).")
        input(
            "[KROK 3] Naciśnij przycisk SET na ESC. Dioda mignie 2 razy. Potem wciśnij [ENTER]..."
        )

        pca.set_servo_pulse(channel, pca.PULSE_MIN, force=True)
        print("-> PCA9685 nadaje sygnał MAX TYŁ (1000us).")
        input(
            "[KROK 4] Naciśnij przycisk SET na ESC. Dioda mignie 3 razy. Potem wciśnij [ENTER]..."
        )

        pca.set_servo_pulse(channel, pca.PULSE_NEUTRAL, force=True)
        print("\n[ZAKOŃCZONO] Wyłącz i włącz ESC. Powinno działać!")

    elif wybor == "2":
        print("\n--- KALIBRACJA AM32 / AUTOMATYCZNA ---")
        print("[KROK 1] Upewnij się, że ESC jest WYŁĄCZONE.")
        pca.set_servo_pulse(channel, pca.PULSE_MAX, force=True)
        print("-> PCA9685 nadaje sygnał MAX PRZÓD (2000us).")
        input(
            "PODŁĄCZ zasilanie do ESC. Poczekaj na pierwsze piknięcie silnika, a następnie SZYBKO wciśnij [ENTER]..."
        )

        pca.set_servo_pulse(channel, pca.PULSE_MIN, force=True)
        print("-> PCA9685 nadaje sygnał MAX TYŁ (1000us).")
        input("Poczekaj na kolejne piknięcie silnika. Potem wciśnij [ENTER]...")

        pca.set_servo_pulse(channel, pca.PULSE_NEUTRAL, force=True)
        print(
            "\n[ZAKOŃCZONO] Sygnał powrócił do neutralnego (1500us). Silnik powinien zagrać melodię uzbrojenia."
        )


def skaner_neutralny(pca: PCA9685, channel: int) -> None:
    """
    Skaner pomagający znaleźć neutralny punkt dla ESC.
    Scanner to help find the neutral point for the ESC.
    """
    sygnal = 1500
    pca.set_servo_pulse(channel, sygnal, force=True)
    print("\n--- SKANER PUNKTU NEUTRALNEGO ---")
    print("Jeśli po włączeniu zasilania ESC cały czas rytmicznie pika, oznacza to,")
    print("że sygnał 1500us nie trafia w 'środek' przez niedokładny zegar na płytce.")
    print("1. Włącz teraz ESC (powinno pikać błędem).")
    print(
        "2. Użyj 'w' (w górę) i 's' (w dół), aby szukać momentu, aż ESC zagra melodię i zamilknie."
    )

    while True:
        print(f"Obecny sygnał: {sygnal} us")
        akcja = input("Akcja [w=Zwiększ o 10 | s=Zmniejsz o 10 | q=Wyjdź]: ").lower()
        if akcja == "w":
            sygnal += 10
            pca.set_servo_pulse(channel, sygnal, force=True)
        elif akcja == "s":
            sygnal -= 10
            pca.set_servo_pulse(channel, sygnal, force=True)
        elif akcja == "q":
            print(f"Zanotuj swój idealny punkt neutralny: {sygnal} us!")
            pca.set_servo_pulse(channel, 1500, force=True)
            break


# ==============================================================================
# MENU GŁÓWNE
# ==============================================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    log = logging.getLogger("MAIN")

    print("\nInicjalizacja układu PCA9685...")
    # Częstotliwość oscylatora 25MHz - typowa. Jeśli skaner wykaże duże odchylenia, zmień to na np. 26000000
    pca_ctrl = PCA9685(logger=log, init_neutral=True, oscillator_freq=25000000)

    if not pca_ctrl.pca:
        print("BŁĄD: Nie znaleziono sprzętu PCA9685. Sprawdź połączenia I2C!")
        sys.exit(1)

    KANAŁ_ESC = 0  # << Zmień jeśli wpinasz ESC w inny kanał niż 0

    try:
        while True:
            print("\n" + "=" * 40)
            print("   PANEL STEROWANIA AUTA RC (PCA9685)")
            print("=" * 40)
            print("1. Kalibracja ESC (Kreator krok po kroku)")
            print("2. Skaner punktu neutralnego (jeśli ESC pika)")
            print("3. Test przepustnicy (Lekki gaz na 1 sek)")
            print("0. Wyjście")

            wybor = input("\nWybierz opcję: ")

            if wybor == "1":
                menu_kalibracji(pca_ctrl, KANAŁ_ESC)
            elif wybor == "2":
                skaner_neutralny(pca_ctrl, KANAŁ_ESC)
            elif wybor == "3":
                print("\nUruchamianie Autokalibracji (wymaga zworki CH15 -> GPIO 17)...")
                pca_ctrl.auto_calibrate = True
                pca_ctrl.autocalibrate_oscillator()
            elif wybor == "4":
                print("Uzbrajanie (1500 us)...")
                pca_ctrl.set_servo_pulse(KANAŁ_ESC, 1500, force=True)
                time.sleep(1)
                print("Delikatny gaz w przód (1580 us)...")
                pca_ctrl.set_servo_pulse(KANAŁ_ESC, 1580, force=True)
                time.sleep(1)
                print("Powrót do neutralnego (1500 us)...")
                pca_ctrl.set_servo_pulse(KANAŁ_ESC, 1500, force=True)
            elif wybor == "0":
                print("Zamykanie...")
                break
            else:
                print("Nieznana opcja.")
    except KeyboardInterrupt:
        print("\nPrzerwano kombinacją klawiszy.")
    finally:
        pca_ctrl.cleanup()
        print("Silniki wyłączone. Program zakończony.")
