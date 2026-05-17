"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Kompozytor struktury Payload z telemetrią.
Agreguje wskazania sensoryczne w ustrukturyzowane obiekty
uwzględniając taktowanie (4Hz dla LiDARu, 1Hz dla GPS).
"""

import base64
import struct
import time
import zlib
from typing import TYPE_CHECKING, Any
from core.version import APP_VERSION

if TYPE_CHECKING:
    from core.main_service import TelemetryWorker


class TelemetryBuilder:
    """
    Klasa odpowiedzialna za budowanie słownika wymiany danych
    wysyłanego do jednostki GCS.
    """

    def __init__(self, worker: "TelemetryWorker"):
        self.worker = worker

    def prepare_telemetry(self, sensor_data: dict[str, Any]) -> dict[str, Any]:
        w = self.worker
        packet = {
            "t": "telemetry",
            "v": APP_VERSION,
            "ts": time.time(),
            "i": w.telemetry_packet_idx,
            "e": w.last_pc_timestamp,
            "mo": w.current_mode,
            "im": sensor_data.get("imu"),
            "po": w.current_pose,
            "acm": w.comm_mode,
            "arm": w.pca_armed,
            "link": w.link_established,
            "sync": w.slam_frame_count,
        }

        # Navigation State
        if w.nav_manager and w.nav_manager.current_path:
            packet["nav"] = {
                "p": w.nav_manager.current_path,
                "wi": w.nav_manager.current_waypoint_idx,
            }

        # LiDAR (Teraz RAW, bez sztucznego limitu przepustowości)
        lidar_scan = sensor_data.get("lidar")
        if lidar_scan:
            distances_mm = [0] * 360
            for angle, dist in lidar_scan:
                try:
                    # [PLAN-007] Robust Casting & Clamping
                    if dist is None:
                        continue

                    idx = int(angle % 360)
                    dist_int = int(dist)

                    if 0 <= idx < 360:
                        # 65535 is max for 'H' (unsigned short)
                        distances_mm[idx] = max(0, min(65535, dist_int))
                except (TypeError, ValueError):
                    # Skip invalid points (e.g. None or non-numeric)
                    continue

            binary_blob = struct.pack("<360H", *distances_mm)
            compressed_blob = zlib.compress(binary_blob, level=6)
            packet["lidar"] = base64.b64encode(compressed_blob).decode("utf-8")
            packet["lidar_compressed"] = (
                True  # Flaga, na którą czeka lidar_processor.py
            )
        else:
            packet["lidar"] = None

        packet["gps"] = sensor_data.get("gps")

        # Bateria i System: Co 20 ramek (1Hz)
        packet["bat"] = sensor_data.get("battery", {})
        packet["sys"] = sensor_data.get("system", {})

        if w.telemetry_packet_idx % 20 == 0:
            if hasattr(w, "slam_manager") and w.slam_manager:
                packet["_internal_grid_bytes"] = w.slam_manager.get_map()

        return packet
