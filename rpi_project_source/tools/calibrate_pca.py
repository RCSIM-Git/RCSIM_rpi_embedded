#!/usr/bin/env python3
"""
RCSIM - Narzędzie do kalibracji oscylatora PCA9685.
RCSIM - PCA9685 Oscillator Calibration Tool.

To narzędzie pozwala na ręczne wywołanie procedury autokalibracji zegara PCA9685.
Wymaga zworki między wybranym kanałem PCA (domyślnie 15) a pinem GPIO (domyślnie 17).
"""

import logging
import sys
import os

# Dodanie ścieżki głównej projektu
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from modules.pca9685 import PCA9685
from modules.drivers.native.native_i2c import I2CWrapper

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("PCA_CALIB")

def run_calibration():
    print("\n--- NARZĘDZIE KALIBRACJI OSCYLATORA PCA9685 ---")
    print("Upewnij się, że masz połączone:")
    print("PCA9685 Channel 15  <--->  Raspberry Pi GPIO 17 (Pin 11)")
    
    confirm = input("\nCzy chcesz kontynuować? [y/N]: ")
    if confirm.lower() != 'y':
        print("Anulowano.")
        return

    try:
        # Inicjalizacja I2C
        i2c = I2CWrapper(bus_num=1)
        
        # Inicjalizacja PCA z flagą auto_calibrate
        pca = PCA9685(
            i2c_bus=i2c,
            logger=logger,
            auto_calibrate=True,
            calib_ch=15,
            calib_gpio=17,
            init_neutral=False
        )
        
        if pca.pca:
            print(f"\nKalibracja zakończona!")
            print(f"Wyliczona częstotliwość oscylatora: {pca.oscillator_freq} Hz")
            print("\nZanotuj tę wartość i wpisz do config.json w sekcji 'pca9685'.")
        else:
            print("\nBŁĄD: Nie udało się zainicjalizować sterownika PCA9685.")
            
    except Exception as e:
        print(f"\nBŁĄD KRYTYCZNY: {e}")
    finally:
        print("\nZamykanie...")

if __name__ == "__main__":
    run_calibration()
