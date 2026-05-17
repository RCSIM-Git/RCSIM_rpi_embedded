"""
Asynchroniczna kolejka do wysyłania komend UDP/WebRTC z RPi.
Zdejmuje operacje kompresji ZLib, kodowania BASE64 i wysyłania
z głównego wątku TelemetryWorker.
"""

import base64
import json
import logging
import threading
import time
import zlib
from queue import Empty, Queue
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from core.udp_service import UDPService
    from core.webrtc_manager import WebRTCManager


class TelemetrySender(threading.Thread):
    """
    Asynchroniczna kolejka do wysyłania komend UDP/WebRTC z RPi.
    Zdejmuje operacje kompresji ZLib, kodowania BASE64 i wysyłania
    z głównego wątku TelemetryWorker.
    """

    def __init__(self, webrtc_service: "WebRTCManager", udp_service: "UDPService", worker: "TelemetryWorker"):
        super().__init__(daemon=True)
        self.queue: Queue[tuple[dict[str, Any], str]] = Queue(maxsize=100)
        self.webrtc_service = webrtc_service
        self.udp_service = udp_service
        self.worker = worker
        self.running = False
        self.logger = logging.getLogger("TelemetrySender")

    def start(self):
        self.running = True
        super().start()

    def stop(self):
        self.running = False

    def send_packet(self, packet: dict[str, Any], channel_label: str = "telemetry"):
        try:
            self.queue.put_nowait((packet, channel_label))
        except Exception:
            pass

    def run(self):
        while self.running:
            try:
                packet, channel_label = self.queue.get(timeout=0.1)

                # [THROTTLING] Adaptive Backpressure for SLAM [NET-005]
                # If network RTT is high, skip heavy map updates
                rtt = time.time() - self.worker.last_pc_timestamp
                is_heavy = "_internal_grid_bytes" in packet or "occupancy_grid_c" in packet
                
                if is_heavy and rtt > 0.4: # 400ms threshold
                    if self.worker.telemetry_packet_idx % 10 != 0: # Only 1/10 maps if slow
                        continue

                # Echo back PC timestamp for RTT calculation on PC side
                packet["rtt_echo"] = self.worker.last_pc_timestamp

                grid_bytes = packet.pop("_internal_grid_bytes", None)

                if grid_bytes is not None:
                    try:
                        compressed = zlib.compress(grid_bytes, level=1)
                        encoded_map = base64.b64encode(compressed).decode("utf-8")

                        if len(encoded_map) > 800:
                            packet["occupancy_grid_c"] = encoded_map
                            from core.chunking import MessageChunker

                            chunks = MessageChunker.chunk_message(packet, max_size=1100)
                            for chunk_str in chunks:
                                self._broadcast(chunk_str, "lidar")
                                time.sleep(0.005)  # Throttle smaller chunks slightly
                        else:
                            packet["occupancy_grid_c"] = encoded_map
                            self._broadcast(json.dumps(packet), channel_label)
                    except Exception as e:
                        self.logger.error(f"Sender Map compression error: {e}")
                else:
                    self._broadcast(json.dumps(packet), channel_label)

            except Empty:
                pass
            except Exception as e:
                self.logger.error(f"Sender loop error: {e}")

    def _broadcast(self, data_str: str, channel_label: str = "telemetry"):
        if self.webrtc_service:
            try:
                self.webrtc_service.send_data(data_str, channel_label)
            except Exception:
                pass
        if self.udp_service:
            try:
                self.udp_service.send_data(data_str)
            except Exception:
                pass
