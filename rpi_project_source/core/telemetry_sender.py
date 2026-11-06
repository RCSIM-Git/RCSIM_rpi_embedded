"""PL: Wysyła telemetrię w tle przez UDP lub WebRTC.

Przenosi kompresję, kodowanie Base64 i transmisję poza główny wątek
``TelemetryWorker``.

EN: Sends telemetry in the background over UDP or WebRTC.

Moves compression, Base64 encoding, and transmission out of the main
``TelemetryWorker`` thread.
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
    """PL: Kolejkuje i wysyła pakiety telemetrii z osobnego wątku.

    EN: Queues and sends telemetry packets from a dedicated thread.
    """

    def __init__(self, webrtc_service: "WebRTCManager", udp_service: "UDPService", worker: "TelemetryWorker"):
        """PL: Konfiguruje kolejkę i usługi używane do transmisji.

        Args:
            webrtc_service: Usługa WebRTC do opcjonalnej wysyłki.
            udp_service: Usługa UDP do opcjonalnej wysyłki.
            worker: Dostawca stanu i znaczników czasu telemetrii.

        EN: Configures the queue and the services used for transmission.

        Args:
            webrtc_service: WebRTC service for optional transmission.
            udp_service: UDP service for optional transmission.
            worker: Provider of telemetry state and timestamps.
        """
        super().__init__(daemon=True)
        self.queue: Queue[tuple[dict[str, Any], str]] = Queue(maxsize=100)
        self.webrtc_service = webrtc_service
        self.udp_service = udp_service
        self.worker = worker
        self.running = False
        self.logger = logging.getLogger("TelemetrySender")

    def start(self):
        """PL: Ustawia stan działania i uruchamia wątek wysyłający.

        EN: Marks the sender as running and starts its worker thread.
        """
        self.running = True
        super().start()

    def stop(self):
        """PL: Sygnalizuje wątkowi wysyłającemu zakończenie pracy.

        EN: Signals the sender thread to stop processing packets.
        """
        self.running = False

    def send_packet(self, packet: dict[str, Any], channel_label: str = "telemetry"):
        """PL: Dodaje pakiet do kolejki bez blokowania wywołującego.

        Args:
            packet: Słownik telemetrii do wysłania.
            channel_label: Etykieta kanału używana przez transport.

        EN: Adds a packet to the queue without blocking the caller.

        Args:
            packet: Telemetry dictionary to send.
            channel_label: Channel label used by the transport.
        """
        try:
            self.queue.put_nowait((packet, channel_label))
        except Exception:
            pass

    def run(self):
        """PL: Pobiera pakiety z kolejki, kompresuje mapę i wysyła dane.

        Przy dużym RTT ogranicza częstotliwość ciężkich aktualizacji mapy.

        EN: Takes queued packets, compresses map data, and transmits the payloads.

        When RTT is high, reduces the rate of heavy map updates.
        """
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
        """PL: Próbuje wysłać tekst przez dostępne usługi WebRTC i UDP.

        Args:
            data_str: Zserializowany pakiet do wysłania.
            channel_label: Etykieta kanału dla transportu WebRTC.

        EN: Attempts to send the text through the available WebRTC and UDP services.

        Args:
            data_str: Serialized packet to send.
            channel_label: Channel label for the WebRTC transport.
        """
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
