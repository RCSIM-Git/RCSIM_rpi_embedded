"""
Moduł pomocniczy do zbierania informacji o systemie i sprzęcie.

Utility module for gathering system and hardware information.
"""

import os
import shutil
import logging
from typing import Any

try:
    import psutil

    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

logger = logging.getLogger(__name__)


def get_storage_info(path: str = "/") -> dict[str, Any]:
    """
    Monitoruje zajętość dysku i liczbę Inodów.
    Monitors disk usage and Inode count.
    """
    try:
        # 1. Bytes usage
        usage = shutil.disk_usage(path)
        used_pct = (usage.used / usage.total) * 100.0

        # 2. Inodes usage (Linux specific)
        inodes_pct = 0.0
        if hasattr(os, "statvfs"):
            st = os.statvfs(path)
            if st.f_files > 0:
                inodes_used = st.f_files - st.f_ffree
                inodes_pct = (inodes_used / st.f_files) * 100.0

        return {
            "used_pct": round(used_pct, 1),
            "inodes_pct": round(inodes_pct, 1),
            "free_gb": round(usage.free / (1024**3), 2),
        }
    except Exception as e:
        logger.error(f"Failed to get storage info: {e}")
        return {"used_pct": 0.0, "inodes_pct": 0.0, "free_gb": 0.0}


def get_board_info() -> dict[str, Any]:
    """
    Zbiera informacje o platformie sprzętowej Raspberry Pi.
    Gathers information about the Raspberry Pi hardware platform.

    Returns:
        dict[str, Any]: Słownik z info o sprzęcie i systemie. / Dictionary with hardware and system info.
    """
    model_name: str = "Unknown"
    is_eco_mode: bool = False
    has_hardware_encoder: bool = False

    try:
        with open("/proc/device-tree/model", "r", encoding="utf-8") as f:
            model_name = f.read().strip()

        # Detekcja trybu "Eco" dla słabszych modeli
        if "Zero 2" in model_name:
            is_eco_mode = True

        # Detekcja sprzętowego enkodera H.264
        # RPi 5 go nie posiada, starsze modele (jak 4, 3, Zero 2) tak.
        if "Raspberry Pi 5" not in model_name:
            has_hardware_encoder = True

    except FileNotFoundError:
        logger.warning("Nie można odczytać modelu płyty z /proc/device-tree/model.")

    cpu_usage: float = 0.0
    ram_usage: float = 0.0
    if PSUTIL_AVAILABLE:
        cpu_usage = psutil.cpu_percent(interval=None)  # Non-blocking
        ram_usage = psutil.virtual_memory().percent

    cpu_temp: float = 0.0
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            cpu_temp = float(f.read().strip()) / 1000.0
    except (FileNotFoundError, ValueError):
        pass

    storage = get_storage_info()

    return {
        "model_name": model_name,
        "is_eco_mode": is_eco_mode,
        "has_hardware_encoder": has_hardware_encoder,
        "cpu_usage": cpu_usage,
        "ram_usage": ram_usage,
        "cpu_temp": cpu_temp,
        "storage": storage,
    }
