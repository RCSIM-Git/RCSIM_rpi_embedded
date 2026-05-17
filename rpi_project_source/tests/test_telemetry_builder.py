"""
Test jednostkowy dla TelemetryBuilder.
Weryfikuje poprawność pakowania binarnego LiDAR (struct -> zlib -> base64).
"""

import base64
import os
import struct
import sys
import unittest
import zlib
from unittest.mock import MagicMock

# Dodaj ścieżkę do modułów
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from core.telemetry_builder import TelemetryBuilder


class TestTelemetryBuilder(unittest.TestCase):
    def setUp(self):
        # Mock TelemetryWorker
        self.worker = MagicMock()
        self.worker.telemetry_packet_idx = 100
        self.worker.last_pc_timestamp = 0
        self.worker.current_mode = "MANUAL"
        self.worker.current_pose = {"x": 0, "y": 0, "yaw": 0}
        self.worker.comm_mode = "WIFI"
        self.worker.pca_armed = True
        self.worker.link_established = True
        self.worker.slam_frame_count = 50
        self.worker.nav_manager = None

        self.builder = TelemetryBuilder(self.worker)

    def test_lidar_packing_and_integrity(self):
        """Weryfikuje skompresowany blob LiDAR (360H)."""
        # 1. Generuj skan: 4 punkty na krzyż
        # (angle, distance)
        raw_scan = [
            (0.5, 1000.7),  # Powinien trafić do idx 0, dystans 1000
            (90.0, 500.0),  # Idx 90, dystans 500
            (180.9, 12000.0),  # Idx 180, dystans 12000
            (270.1, 0.0),  # Idx 270, dystans 0
        ]

        sensor_data = {"lidar": raw_scan, "imu": {}, "gps": {}}

        # 2. Pakuj
        packet = self.builder.prepare_telemetry(sensor_data)

        self.assertIn("lidar", packet)
        self.assertTrue(packet["lidar_compressed"])

        # 3. Dekoduj (PC Side simulation)
        b64_data = packet["lidar"]
        compressed = base64.b64decode(b64_data)
        decompressed = zlib.decompress(compressed)

        # Oczekujemy 360 * 2 bajty (unsigned short) = 720 bajtów
        self.assertEqual(len(decompressed), 720)

        unpacked = struct.unpack("<360H", decompressed)

        # 4. Assert
        self.assertEqual(unpacked[0], 1000)
        self.assertEqual(unpacked[90], 500)
        self.assertEqual(unpacked[180], 12000)
        self.assertEqual(unpacked[270], 0)

        # Sprawdź czy pozostałe są zerami
        self.assertEqual(unpacked[45], 0)

    def test_lidar_clamping(self):
        """Sprawdza odporność na ekstremalne wartości."""
        # Wartości powyżej 65535 i poniżej 0
        raw_scan = [(10.0, 70000), (20.0, -500), (30.0, None)]
        sensor_data = {"lidar": raw_scan}

        # Ten test powinien przejść bez błędu struct.error
        try:
            packet = self.builder.prepare_telemetry(sensor_data)
            self.assertIsNotNone(packet["lidar"])
        except Exception as e:
            self.fail(f"TelemetryBuilder failed on invalid data: {e}")


if __name__ == "__main__":
    unittest.main()
