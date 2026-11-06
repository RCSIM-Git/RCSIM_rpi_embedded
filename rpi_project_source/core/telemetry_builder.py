"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Buduje strukturę ładunku telemetrii, agregując dane sensoryczne i stan
systemu zgodnie z częstotliwością próbkowania poszczególnych źródeł.

EN: Builds the telemetry payload by aggregating sensor data and system state
according to the sampling rate of each data source.
"""

import base64
import struct
import time
import zlib
from typing import Any
from core.version import APP_VERSION


class TelemetryBuilder:
    """PL: Przygotowuje słownik telemetrii wysyłany do stacji GCS.

    EN: Prepares the telemetry dictionary sent to the GCS.
    """

    def __init__(self, worker: "TelemetryWorker"):
        """PL: Zapisuje worker dostarczający bieżący stan systemu.

        EN: Stores the worker that provides the current system state.
        """
        self.worker = worker

    def prepare_telemetry(self, sensor_data: dict[str, Any]) -> dict[str, Any]:
        """PL: Tworzy pakiet telemetrii z danych czujników i stanu workera.

        Dane LiDAR są kodowane jako skompresowana mapa odległości; metadane
        nawigacji, baterii i systemu są dołączane, jeśli są dostępne.

        Args:
            sensor_data: Bieżące odczyty czujników, w tym IMU, LiDAR i GPS.

        Returns:
            Słownik gotowy do serializacji i wysłania do GCS.

        EN: Builds a telemetry packet from sensor readings and worker state.

        LiDAR data is encoded as a compressed distance map; navigation, battery,
        and system metadata are included when available.

        Args:
            sensor_data: Current sensor readings, including IMU, LiDAR, and GPS.

        Returns:
            A dictionary ready to serialize and send to the GCS.
        """
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
            "sp": getattr(w, "current_speed", 0.0),
        }
        imu = sensor_data.get("imu")
        if imu:
            packet["im"] = dict(imu)
            packet["rpi_orientation"] = {
                key: packet["im"].pop(key) for key in ("roll", "pitch", "yaw")
                if key in packet["im"]
            }
            packet["rpi_orientation"].update(source="rpi_complementary", angle_unit="deg")
            sample_time = imu.get("sample_time_us")
            if isinstance(sample_time, (int, float)):
                packet["im"]["sample_age_ms"] = max(
                    0.0, (time.monotonic_ns() / 1000 - sample_time) / 1000)

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
