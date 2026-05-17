"""
Centralny rejestr sensorów IMU (Registry Pattern).
Central IMU sensor registry (Registry Pattern).

Sensory rejestrują się samodzielnie poprzez dekorator @SensorRegistry.register.
Sensors self-register via @SensorRegistry.register decorator.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from modules.drivers.base_sensor import IMUBase
    from modules.drivers.native.native_i2c import I2CWrapper

logger = logging.getLogger(__name__)


class SensorRegistry:
    """
    Centralny rejestr klas sensorów IMU.
    Central registry for IMU sensor classes.

    Drivery rejestrują się automatycznie przy imporcie za pomocą
    dekoratora ``@SensorRegistry.register``.
    Drivers register automatically on import via
    ``@SensorRegistry.register`` decorator.
    """

    _registry: list[type[IMUBase]] = []

    @classmethod
    def register(cls, sensor_class: type[IMUBase]) -> type[IMUBase]:
        """
        Dekorator rejestrujący klasę sensora w globalnym rejestrze.
        Decorator that registers a sensor class in the global registry.

        Args:
            sensor_class: Klasa sensora do zarejestrowania.
                          Sensor class to register.

        Returns:
            type[IMUBase]: Ta sama klasa (niezmieniona).
                           The same class (unmodified).
        """
        if sensor_class not in cls._registry:
            cls._registry.append(sensor_class)
            # Sortuj wg priorytetu — niższy = sprawdzany pierwszy
            # Sort by priority — lower = checked first
            cls._registry.sort(key=lambda c: c.PRIORITY)
            logger.debug(
                f"Registered sensor: {sensor_class.DRIVER_NAME} "
                f"(priority={sensor_class.PRIORITY})"
            )
        return sensor_class

    @classmethod
    def detect(cls, i2c: I2CWrapper) -> type[IMUBase] | None:
        """
        Skanuje magistralę I2C i zwraca pierwszy pasujący driver.
        Scans the I2C bus and returns the first matching driver.

        Drivery są sprawdzane w kolejności priorytetu (niższy
        priorytet = wyższy ranking).
        Drivers are checked in priority order (lower
        priority value = higher ranking).

        Args:
            i2c: Wrapper magistrali I2C. / I2C bus wrapper.

        Returns:
            type[IMUBase] | None: Klasa sensora lub None.
                                  Sensor class or None.
        """
        cls._ensure_drivers_loaded()
        for sensor_class in cls._registry:
            try:
                if sensor_class.scan(i2c):
                    logger.info(
                        f"Detected: {sensor_class.DRIVER_NAME} "
                        f"(priority={sensor_class.PRIORITY})"
                    )
                    return sensor_class
            except Exception as e:
                logger.debug("Scan error for " f"{sensor_class.DRIVER_NAME}: {e}")
        return None

    @classmethod
    def get_by_name(cls, driver_name: str) -> type[IMUBase] | None:
        """
        Zwraca klasę sensora po nazwie sterownika.
        Returns a sensor class by driver name.

        Args:
            driver_name: Nazwa sterownika, np. 'native_bno08x'.
                         Driver name, e.g. 'native_bno08x'.

        Returns:
            type[IMUBase] | None: Klasa sensora lub None.
                                  Sensor class or None.
        """
        cls._ensure_drivers_loaded()
        for sensor_class in cls._registry:
            if sensor_class.DRIVER_NAME == driver_name:
                return sensor_class
        return None

    @classmethod
    def all_drivers(cls) -> list[type[IMUBase]]:
        """
        Zwraca listę wszystkich zarejestrowanych driverów.
        Returns list of all registered drivers.
        """
        cls._ensure_drivers_loaded()
        return list(cls._registry)

    @classmethod
    def _ensure_drivers_loaded(cls) -> None:
        """
        Ładuje moduły driverów, aby dekoratory @register się wykonały.
        Loads driver modules so that @register decorators fire.

        Wspiera ładowanie wbudowanych sterowników oraz zewnętrznych
        modułów z folderu 'workshop/'.
        Supports loading built-in drivers and external modules
        from the 'workshop/' folder.
        """
        if cls._registry:
            return  # Już załadowane / Already loaded

        import importlib
        import os
        import sys

        # 1. Załaduj wbudowane drivery / Load built-in drivers
        _native_modules = [
            "modules.drivers.native.native_bno08x",
            "modules.drivers.native.native_bmx160_bmp388",
            "modules.drivers.native.native_bmx160",
            "modules.drivers.native.native_gy91",
            "modules.drivers.native.native_mpu9250",
            "modules.drivers.native.native_mpu6050",
        ]

        for mod_name in _native_modules:
            try:
                importlib.import_module(mod_name)
            except ImportError as e:
                logger.debug(f"Could not load native driver {mod_name}: {e}")

        # 2. Załaduj drivery ze Steam Workshop / Load Steam Workshop drivers
        # Domyślna ścieżka: /app/workshop lub relatywnie do projektu
        workshop_dir = os.path.join(os.getcwd(), "modules", "drivers", "workshop")
        
        if os.path.exists(workshop_dir):
            logger.info(f"Scanning workshop directory: {workshop_dir}")
            
            # Upewnij się, że folder jest w sys.path dla importów
            if workshop_dir not in sys.path:
                sys.path.append(workshop_dir)

            for filename in os.listdir(workshop_dir):
                if filename.endswith(".py") and not filename.startswith("__"):
                    mod_name = filename[:-3]
                    try:
                        # Ładujemy jako moduł relatywny do drivers.workshop
                        full_mod_name = f"modules.drivers.workshop.{mod_name}"
                        importlib.import_module(full_mod_name)
                        logger.info(f"Loaded workshop driver: {mod_name}")
                    except Exception as e:
                        logger.error(f"Failed to load workshop driver {mod_name}: {e}")
        else:
            # Stwórz folder jeśli nie istnieje, aby ułatwić użytkownikowi
            try:
                os.makedirs(workshop_dir, exist_ok=True)
                # Dodaj __init__.py aby folder był pakietem
                with open(os.path.join(workshop_dir, "__init__.py"), "w") as f:
                    f.write('"""Workshop drivers package."""\n')
            except Exception as e:
                logger.debug(f"Could not create workshop directory: {e}")
