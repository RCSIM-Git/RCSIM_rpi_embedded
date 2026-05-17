"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Moduł obsługi komunikacji UDP.
UDP Communication Handler Module.
"""

import logging
import socket
import threading
from typing import Callable


class UDPService:
    """
    Serwis komunikacji UDP dla RPi (Legacy Mode).
    UDP Communication Service for RPi (Legacy Mode).

    Wymienia pakiety JSON:
    - Odbiera: Kontrolę (sterowanie + komendy)
    - Wysyła: Telemetrię
    Exchanges JSON packets:
    - Receives: Control (steering + commands)
    - Sends: Telemetry
    """

    def __init__(
        self,
        port: int,
        on_data_received: Callable[[str], None],
        target_ip: str | None = None,
        target_port: int | None = None,
    ):
        """
        Inicjalizuje serwis UDP.
        Initializes the UDP service.

        Args:
            port (int): Port nasłuchujący. / Listening port.
            on_data_received (Callable[[str], None]): Callback dla danych przychodzących.
                                                      Callback for incoming data.
            target_ip (str | None): IP docelowe. / Target IP.
            target_port (int | None): Port docelowy. / Target port.
        """
        self.port = port
        self.callback = on_data_received
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Re-use address to avoid bind errors on restart
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))

        self.running = False

        # Pre-set target if provided (allows sending before receiving)
        if target_ip and target_port:
            self.last_client_addr = (target_ip, target_port)
            self.logger = logging.getLogger("UDPService")
            self.logger.info(f"Target set to {target_ip}:{target_port}")
        else:
            self.last_client_addr = None
            self.logger = logging.getLogger("UDPService")

        self._thread: threading.Thread | None = None

    def start(self):
        """
        Uruchamia wątek nasłuchujący.
        Starts the listening thread.
        """
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.logger.info(f"UDP Service started on port {self.port}")

    def stop(self):
        """
        Zatrzymuje serwis.
        Stops the service.
        """
        self.running = False
        if self.sock:
            # Force unblock recv via shutdown/close logic or just cleanup
            try:
                self.sock.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=1.0)
        self.logger.info("UDP Service stopped")

    def send_data(self, data: str):
        """
        Wysyła dane do ostatniego znanego klienta.
        Sends data to the last known client.

        Args:
            data (str): Dane do wysłania. / Data to send.
        """
        if self.last_client_addr and self.sock:
            try:
                if isinstance(data, str):
                    payload = data.encode("utf-8")
                else:
                    payload = data
                self.sock.sendto(payload, self.last_client_addr)
            except Exception as e:
                self.logger.error(f"UDP Send Error: {e}")

    def _loop(self):
        """
        Pętla odbiorcza UDP.
        UDP receive loop.
        """
        while self.running:
            try:
                # Use select or timeout to allow clean exit
                self.sock.settimeout(1.0)
                try:
                    data, addr = self.sock.recvfrom(4096)
                    self.last_client_addr = addr

                    if self.callback:
                        self.callback(data)
                except socket.timeout:
                    continue
                except OSError:
                    # Socket closed
                    break
            except Exception as e:
                if self.running:
                    self.logger.error(f"UDP Loop Error: {e}")
