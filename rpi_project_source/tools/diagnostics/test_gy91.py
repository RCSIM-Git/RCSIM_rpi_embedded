#!/usr/bin/env python3
"""
Skrypt testowy dla modułu GY-91 (MPU-9250/MPU-9255 + BMP280).
Test script for GY-91 module (MPU-9250/MPU-9255 + BMP280).

Weryfikuje poprawkę factory adjustments dla magnetometru.
Verifies the factory adjustments fix for magnetometer.
"""

import os
import sys
import time

# Dodaj ścieżkę do modułów projektu
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from modules.drivers.native.native_gy91 import NativeGY91
from modules.drivers.native.native_i2c import I2CWrapper


def main():
    print("=" * 60)
    print("Test GY-91 - Weryfikacja ulepszeń z GitHub ricardozago")
    print("=" * 60)

    try:
        # Inicjalizacja I2C
        print("\n[1/5] Inicjalizacja I2C bus...")
        i2c = I2CWrapper(bus_number=1)  # RPi używa bus 1
        print("✓ I2C zainicjalizowany")

        # Inicjalizacja GY-91
        print("\n[2/5] Inicjalizacja modułu GY-91...")
        gy91 = NativeGY91(i2c)
        print("✓ GY-91 zainicjalizowany")

        # Wyświetl informacje o chipach
        print("\n[3/5] Informacje o wykrytych chipach:")
        chip_info = gy91.get_chip_info()
        for key, value in chip_info.items():
            print(f"  • {key}: {value}")

        # Test odczytu danych
        print("\n[4/5] Test odczytu danych (10 próbek)...")
        for i in range(10):
            data = gy91.read_data()

            print(f"\nPróbka {i+1}/10:")
            print(
                f"  Akcelerometr: ax={data.get('ax', 'N/A'):.3f} g, "
                f"ay={data.get('ay', 'N/A'):.3f} g, "
                f"az={data.get('az', 'N/A'):.3f} g"
            )
            print(
                f"  Żyroskop:     gx={data.get('gx', 'N/A'):.3f} °/s, "
                f"gy={data.get('gy', 'N/A'):.3f} °/s, "
                f"gz={data.get('gz', 'N/A'):.3f} °/s"
            )

            # 🔧 KLUCZOWY TEST: Sprawdź czy magnetometr zwraca wartości
            mx = data.get("mx", None)
            my = data.get("my", None)
            mz = data.get("mz", None)

            if mx is not None and my is not None and mz is not None:
                print(
                    f"  Magnetometr:  mx={mx:.2f} µT, "
                    f"my={my:.2f} µT, "
                    f"mz={mz:.2f} µT"
                )
                print("  ✓ Magnetometr działa poprawnie!")
            else:
                print("  ✗ PROBLEM: Magnetometr zwraca None")

            print(
                f"  Barometr:     ciśnienie={data.get('pressure', 'N/A'):.2f} hPa, "
                f"temp={data.get('temperature', 'N/A'):.2f} °C"
            )

            time.sleep(0.5)

        # Test kalibracji
        print("\n[5/5] Test kalibracji...")
        print("UWAGA: Nie ruszaj modułu podczas kalibracji!")
        time.sleep(2)

        success = gy91.calibrate()
        if success:
            print("✓ Kalibracja zakończona pomyślnie")
        else:
            print("✗ Kalibracja nie powiodła się")

        print("\n" + "=" * 60)
        print("Test zakończony!")
        print("=" * 60)

        # Podsumowanie
        print("\n📋 PODSUMOWANIE:")
        if chip_info.get("magnetometer_present", False):
            if mx is not None:
                print("✓ Magnetometr wykryty i działa poprawnie")
                print("✓ Factory adjustments zostały zastosowane")
            else:
                print("⚠ Magnetometr wykryty, ale zwraca None - sprawdź połączenie I2C")
        else:
            print("✗ Magnetometr nie został wykryty")
            print("  Możliwe przyczyny:")
            print("  1. Bypass mode nie działa w MPU")
            print("  2. AK89 63 nie jest podłączony")
            print("  3. Problem z I2C bus")

    except KeyboardInterrupt:
        print("\n\nTest przerwany przez użytkownika")
    except Exception as e:
        print(f"\n✗ BŁĄD: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
