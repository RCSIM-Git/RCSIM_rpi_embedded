"""
Moduł składania binarnych pakietów ścieżki (PT - Path Transmission) na RPi.
Reassembles binary path packets (PT) into waypoint lists.
"""

import logging
import struct
import time
from typing import Any


class BinaryPathAssembler:
    """
    Składa fragmenty ścieżki przesyłane binarnie z GCS.
    Assembles path fragments sent in binary format from GCS.
    """

    def __init__(self):
        self.logger = logging.getLogger("BinaryPathAssembler")
        self._buffers: dict[int, dict] = {}
        self._last_cleanup = time.time()

    def add_chunk(self, data: bytes) -> list[dict[str, Any]] | None:
        """
        Dodaje fragment PT i zwraca kompletną ścieżkę jeśli zmontowana.
        Format (10B Header): PT + ID(2) + Index(2) + Total(2) + Len(2)
        Data: [lat:f32, lon:f32, alt:f32, type:u8] * N
        Trailer (1B): CRC
        """
        if len(data) < 11 or not data.startswith(b"PT"):
            return None

        # 1. Walidacja CRC (XOR)
        received_crc = data[-1]
        calculated_crc = 0
        for b in data[:-1]:
            calculated_crc ^= b

        if calculated_crc != received_crc:
            self.logger.warning("Path Chunk CRC Error")
            return None

        # 2. Dekodowanie Nagłówka
        header = data[2:10]
        msg_id, index, total, payload_len = struct.unpack("<HHHH", header)

        chunk_payload = data[10:-1]
        if len(chunk_payload) != payload_len:
            self.logger.error(
                f"Payload length mismatch: {len(chunk_payload)} != {payload_len}"
            )
            return None

        # 3. Buforowanie
        if msg_id not in self._buffers:
            self._buffers[msg_id] = {
                "chunks": {},
                "total": total,
                "last_seen": time.time(),
            }

        buffer = self._buffers[msg_id]
        buffer["chunks"][index] = chunk_payload
        buffer["last_seen"] = time.time()

        # 4. Sprawdzenie Kompletności
        if len(buffer["chunks"]) == buffer["total"]:
            self.logger.info(
                f"Binary path {msg_id} fully reassembled ({total} chunks)."
            )
            full_data = b"".join(buffer["chunks"][i] for i in range(total))
            del self._buffers[msg_id]
            return self._parse_path_bytes(full_data)

        # 5. Okresowe czyszczenie starych buforów
        if time.time() - self._last_cleanup > 30:
            self.cleanup()

        return None

    def _parse_path_bytes(self, data: bytes) -> list[dict[str, Any]]:
        """Konwertuje surowe bajty na listę waypointów (format MissionManager)."""
        waypoints = []
        point_size = 13  # 3*f32(4B) + 1*u8(1B)
        num_points = len(data) // point_size

        for i in range(num_points):
            offset = i * point_size
            lat, lon, alt, wp_type = struct.unpack(
                "<fffB", data[offset : offset + point_size]
            )
            waypoints.append(
                {
                    "lat": float(lat),
                    "lon": float(lon),
                    "alt": float(alt),
                    "type": int(wp_type),
                }
            )
        return waypoints

    def cleanup(self, timeout: float = 10.0):
        """Usuwa niekompletne, przedawnione transmisje."""
        now = time.time()
        expired = [
            mid for mid, b in self._buffers.items() if now - b["last_seen"] > timeout
        ]
        for mid in expired:
            del self._buffers[mid]
            self.logger.warning(f"Discarding incomplete binary path transmission {mid}")
        self._last_cleanup = now
