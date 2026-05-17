"""
Pytest configuration file for the RPi project.
Plik konfiguracyjny Pytest dla projektu RPi.

This file defines fixtures that are automatically discovered and used by pytest.
The `autouse=True` fixtures are applied to all tests in the suite.

Ten plik definiuje fixture'y, które są automatycznie wykrywane i używane przez pytest.
Fixture'y z `autouse=True` są stosowane do wszystkich testów w pakiecie.
"""

import sys
from unittest.mock import MagicMock

import pytest

# MOCKING AT IMPORT TIME (Before collection)
# Mockowanie przy imporcie (Przed zbieraniem testów)
hardware_modules = [
    "board",
    "busio",
    "smbus2",
    "serial",
    "picamera2",
    "adafruit_pca9685",
    "psutil",
]

for module_name in hardware_modules:
    sys.modules[module_name] = MagicMock()


@pytest.fixture(autouse=True)
def mock_rpi_hardware(monkeypatch):
    """
    Ensures mocks are maintained during tests (redundant but safe).
    Zapewnia utrzymanie mocków podczas testów (nadmiarowe, ale bezpieczne).
    """
    for module_name in hardware_modules:
        if module_name not in sys.modules:
            sys.modules[module_name] = MagicMock()
