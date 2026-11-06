"""PL: Konfiguracja pytest i mocki zależności sprzętowych projektu RPI.

EN: Pytest configuration and hardware-dependency mocks for the RPI project.
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
    """PL: Zapewnia dostępność mocków sprzętowych podczas każdego testu.

    EN: Ensure hardware mocks remain available during each test.
    """
    for module_name in hardware_modules:
        if module_name not in sys.modules:
            sys.modules[module_name] = MagicMock()
