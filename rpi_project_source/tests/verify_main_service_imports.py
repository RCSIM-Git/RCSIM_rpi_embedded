import os
import sys
from unittest.mock import MagicMock

# Mock RPi modules
sys.modules["RPi"] = MagicMock()
sys.modules["RPi.GPIO"] = MagicMock()
sys.modules["board"] = MagicMock()
sys.modules["busio"] = MagicMock()
sys.modules["picamera"] = MagicMock()
sys.modules["serial"] = MagicMock()
sys.modules["hailort"] = MagicMock()  # Mock Hailo since we might not have it

# Add project root to path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

print(f"Project root: {project_root}")

try:
    from core.main_service import TelemetryWorker

    print("Successfully imported TelemetryWorker.")

    # Try instantiation with dummy config
    config = {
        "main_loop_freq_hz": 20,
        "comm_mode": "UDP",
        "camera": {"enabled": False},  # Disable camera for test
        "hardware": {"mock": True},  # Use mock hardware if supported
        "ai": {"enabled": False},
        "slam": {"enabled": False},
    }

    worker = TelemetryWorker(config)
    print("Successfully instantiated TelemetryWorker.")

except Exception as e:
    print(f"Import/Instantiation failed: {e}")
    sys.exit(1)

print("Verification complete.")
