import os
import sys

# Add the project root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

module = "modules.managers.hardware_manager"

print(f"Verifying {module}...")
try:
    __import__(module)
    print(f"OK: {module}")
except ImportError as e:
    print(f"FAILED: {module} - {e}")
except Exception as e:
    print(f"ERROR: {module} - {e}")
