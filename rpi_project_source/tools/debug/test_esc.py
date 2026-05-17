#!/usr/bin/env python3
"""
ESC Calibration and Test Tool for RCSIM
Warstwa testowa i kalibracyjna dla ESC (Electronic Speed Controllers).

Ten skrypt pozwala na:
1. Sprawdzenie działania sterownika PCA9685 (Native lub Adafruit).
2. Ustawienie częstotliwości PWM (domyślnie 50Hz).
3. Ręczne sterowanie szerokością impulsu (1000-2000us) dla wybranego kanału.
4. Przeprowadzenie procedury kalibracji ESC (Max -> Min -> Neutral).
"""

import logging
import sys
import time

try:
    from modules.pca9685 import PCA9685

    WRAPPER_AVAILABLE = True
except ImportError:
    WRAPPER_AVAILABLE = False

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("ESC_TEST")


class ESCTester:
    def __init__(self):
        self.pca_wrapper = None
        self.address = 0x40
        self.frequency = 50
        self.init_driver()

    def init_driver(self):
        logger.info("Inicjalizacja sterownika PCA9685 via Wrapper...")
        if not WRAPPER_AVAILABLE:
            logger.critical(
                "Nie można zaimportować modules.pca9685. Sprawdź PYTHONPATH."
            )
            sys.exit(1)

        try:
            # Używamy ujednoliconego wrappera
            self.pca_wrapper = PCA9685(
                address=self.address,
                frequency=self.frequency,
                init_neutral=False,  # Zaczynamy w trybie Silent dla bezpieczeństwa
            )
            if self.pca_wrapper.pca:
                logger.info(
                    f"Sterownik zainicjalizowany: {self.pca_wrapper.driver_type}"
                )
            else:
                logger.error("Inicjalizacja nieudana (Mock Mode).")
        except Exception as e:
            logger.critical(f"Błąd inicjalizacji: {e}")
            sys.exit(1)

    def set_freq(self, freq):
        self.frequency = freq
        if self.pca_wrapper:
            # Wrapper nie ma metody set_frequency (robi to przy init),
            # ale możemy ją dodać lub zreinicjalizować.
            # Tutaj bezpośrednio do obiektu sterownika:
            if self.pca_wrapper.driver_type == "Native":
                self.pca_wrapper.pca.set_frequency(freq)
            elif self.pca_wrapper.driver_type == "Adafruit":
                self.pca_wrapper.pca.frequency = freq
            self.pca_wrapper._frequency = freq

        logger.info(f"Częstotliwość ustawiona na {freq} Hz")
        self.set_all_neutral()

    def set_pulse(self, channel, pulse_us, force=False):
        if self.pca_wrapper:
            self.pca_wrapper.set_servo_pulse(channel, pulse_us, force=force)
        logger.debug(f"Ch {channel} -> {pulse_us} us (force={force})")

    def set_all_neutral(self):
        logger.info("Ustawianie wszystkich kanałów na Neutral (1500us)...")
        if self.pca_wrapper:
            self.pca_wrapper.set_all_channels_neutral()

    def calibrate_esc(self, channel):
        print(f"\n--- INTERAKTYWNA KALIBRACJA ESC (Kanał {channel}) ---")
        print("UWAGA: Zdejmij śmigła lub unieś koła pojazdu!")
        print(
            "Procedura: MAX -> Podłącz baterię -> Czekaj na bip -> MIN -> Czekaj -> NEUTRAL"
        )

        input("1. Odłącz baterię od ESC. Naciśnij ENTER, aby kontynuować...")

        print("2. Ustawiam gaz na MAX (2000us).")
        self.set_pulse(channel, 2000, force=True)

        print("3. Podłącz teraz baterię do ESC.")
        print("   Powinieneś usłyszeć dźwięki informujące o wejściu w tryb kalibracji.")
        input("   Naciśnij ENTER gdy usłyszysz sekwencję dźwięków MAX...")

        print("4. Ustawiam gaz na MIN (1000us).")
        self.set_pulse(channel, 1000, force=True)

        print("5. Czekam na potwierdzenie MIN (ok. 2 sekundy)...")
        time.sleep(2)

        print("6. Ustawiam gaz na NEUTRAL (1500us).")
        self.set_pulse(channel, 1500)

        print("Procedura zakończona. Sprawdź, czy silnik reaguje.")

    def manual_control(self, channel):
        print(f"\n--- RĘCZNA KONTROLA (Kanał {channel}) ---")
        print("Wpisuj wartości impulsów w us (800-2200). 'q' aby wyjść.")
        while True:
            val = input("Impuls [800-2200]: ")
            if val.lower() == "q":
                break
            try:
                pulse = int(val)
                # Używamy force=True w narzędziu testowym, wrapper i tak ma clamp w set_servo_pulse
                # ale jako narzędzie debugowe chcemy mieć pełną kontrolę
                self.set_pulse(channel, pulse, force=True)
            except ValueError:
                print("Błędna wartość.")

    def run(self):
        print("--- RCSIM ESC TOOL ---")
        # Domyślnie ustaw 50Hz i Neutral na wszystkie kanały
        try:
            self.set_freq(50)
        except Exception as e:
            logger.error(f"Błąd ustawiania częstotliwości: {e}")

        self.set_all_neutral()

        while True:
            print("\nMENU:")
            print("1. Ustaw Częstotliwość PWM (obecnie: 50Hz)")
            print("2. Test Ręczny Kanału (Manual Pulse)")
            print("3. Procedura Kalibracji ESC (Max->Min->Neutral)")
            print("4. Wyślij Neutral (1500us) na wszystkie kanały")
            print("q. Wyjście")

            choice = input("Wybór: ")

            if choice == "1":
                try:
                    freq = int(
                        input("Podaj częstotliwość [40-1000 Hz, domyślnie 50]: ")
                    )
                    self.set_freq(freq)
                except ValueError:
                    print("Błąd liczby.")

            elif choice == "2":
                try:
                    ch = int(input("Numer kanału [0-15]: "))
                    self.manual_control(ch)
                except ValueError:
                    print("Błąd kanału.")

            elif choice == "3":
                try:
                    ch = int(input("Numer kanału ESC [0-15]: "))
                    self.calibrate_esc(ch)
                except ValueError:
                    print("Błąd kanału.")

            elif choice == "4":
                self.set_all_neutral()

            elif choice.lower() == "q":
                print("Zamykanie...")
                self.set_all_neutral()
                break


if __name__ == "__main__":
    tester = ESCTester()
    tester.run()
