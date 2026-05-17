"""
RCSIM Workshop - Przykładowy Sterownik IMU (Starter Kit)
RCSIM Workshop - Example IMU Driver (Starter Kit)

Ten plik służy jako referencja dla twórców społecznościowych.
Został zaprojektowany tak, aby pokazać jak mapować surowe dane na standard RCSIM.
This file serves as a reference for community creators.
It's designed to show how to map raw data to the RCSIM standard.
"""

from modules.drivers.base_sensor import IMUBase
from modules.drivers.sensor_registry import SensorRegistry
import logging
import random # Tylko dla potrzeb symulacji w starter kicie

logger = logging.getLogger(__name__)

@SensorRegistry.register
class StarterKitIMU(IMUBase):
    """
    Przykładowa klasa sterownika. 
    System automatycznie wykryje ten plik w folderze 'workshop/'.
    """
    
    DRIVER_NAME = "workshop_starter_kit"
    
    # Przykładowe adresy I2C (np. dla MPU6050 lub BNO055)
    I2C_ADDRESSES = [0x68, 0x28]
    
    PRIORITY = 200 # Niższy priorytet niż drivery natywne

    def __init__(self, i2c_wrapper):
        """
        Inicjalizacja sensora.
        i2c_wrapper udostępnia metody read_byte_data, write_byte_data itp.
        """
        self.i2c = i2c_wrapper
        logger.info(f"[{self.DRIVER_NAME}] Inicjalizacja Starter Kit IMU...")
        
        # Przykład: Sprawdzenie ID urządzenia (WHO_AM_I)
        try:
            # device_id = self.i2c.read_byte_data(0x68, 0x75)
            # logger.info(f"Wykryto Device ID: {hex(device_id)}")
            pass
        except Exception as e:
            logger.warning(f"Błąd inicjalizacji: {e}")

    def read_data(self) -> dict:
        """
        Główna metoda odczytu danych.
        Musi zwracać słownik z ustandaryzowanymi kluczami.
        """
        # W prawdziwym sterowniku tutaj odczytałbyś rejestry I2C
        # In a real driver, you would read I2C registers here
        
        return {
            "ax": 0.0 + random.uniform(-0.1, 0.1), 
            "ay": 0.0 + random.uniform(-0.1, 0.1), 
            "az": 9.81 + random.uniform(-0.1, 0.1),
            "gx": 0.0, 
            "gy": 0.0, 
            "gz": 0.0,
            "mx": 0.0, 
            "my": 0.0, 
            "mz": 0.0,
            "temp": 28.5
        }

    def calibrate(self) -> bool:
        """
        Logika kalibracji (np. zerowanie żyroskopu).
        """
        logger.info(f"[{self.DRIVER_NAME}] Kalibracja zakończona sukcesem.")
        return True
