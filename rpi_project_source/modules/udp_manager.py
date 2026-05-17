"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import logging
import socket
import threading
import time
from typing import Callable, Optional


class UDPManager:
    """
    Zarządza komunikacją UDP dla RPi.
    Manages UDP communication for the RPi.

    Nasłuchuje pakietów kontrolnych i wysyła telemetrię z powrotem do nadawcy.
    Listens for control packets and sends telemetry back to the sender.
    """

    def __init__(
        self,
        port: int,
        on_data_received: Callable[[str], None],
        on_timeout: Optional[Callable[[], None]] = None,
    ):
        """
        Inicjalizuje UDPManager.
        Initializes the UDP Manager.

        Args:
            port (int): Port nasłuchiwania. / Listening port.
            on_data_received (Callable[[str], None]): Funkcja wywoływana po otrzymaniu danych.
            on_timeout (Callable[[], None]): Funkcja wywoływana przy utracie łączności.
        """
        self.port = port
        self.callback = on_data_received
        self.on_timeout = on_timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to all interfaces
        self.sock.bind(("0.0.0.0", self.port))
        self.running = False
        self.last_addr = None
        self.logger = logging.getLogger("UDPManager")

        # [PLAN-006] Heartbeat & Throttling
        self.last_recv_time = time.time()
        self.last_send_time = 0.0
        self.min_send_interval = 0.033  # ~30Hz Max Telemetry Rate
        self.timeout_threshold = 1.0  # 1s Heartbeat Timeout
        self.is_connected = False

    def start(self):
        """
        Uruchamia wątek nasłuchujący UDP.
        Starts the UDP listening thread.
        """
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.logger.info(f"UDP Listener started on port {self.port}")

    def _loop(self):
        """
        Główna pętla wątku nasłuchującego.
        Main listener thread loop.
        """
        while self.running:
            try:
                # Set timeout to allow check for self.running
                self.sock.settimeout(1.0)
                try:
                    data, addr = self.sock.recvfrom(4096)

                    # Update statuses
                    if self.last_addr != addr:
                        self.last_addr = addr
                        self.logger.info(f"New client connected: {addr}")

                    self.last_recv_time = time.time()
                    if not self.is_connected:
                        self.is_connected = True
                        self.logger.info("UDP Heartbeat: Link Established.")

                    if self.callback:
                        # Decode and pass to callback
                        try:
                            decoded_data = data.decode("utf-8")
                            self.callback(decoded_data)
                        except UnicodeDecodeError:
                            self.logger.warning("Received non-UTF8 data")

                except socket.timeout:
                    # [PLAN-006] Check for Heartbeat Timeout
                    if self.is_connected and (
                        time.time() - self.last_recv_time > self.timeout_threshold
                    ):
                        self.logger.warning("UDP Heartbeat: Link LOST (Timeout)!")
                        self.is_connected = False
                        if self.on_timeout:
                            self.on_timeout()
                    continue

            except Exception as e:
                if self.running:
                    self.logger.error(f"UDP Loop Error: {e}")

    def send_data(self, data: str, force: bool = False):
        """
        Wysyła dane (ciąg JSON) z powrotem do ostatnio znanego klienta.
        Sends data (JSON string) back to the last known client.
        """
        if not self.last_addr:
            return

        # [PLAN-006] Throttling: Skip telemetry if sent too fast
        now = time.time()
        if not force and (now - self.last_send_time < self.min_send_interval):
            return

        try:
            # Ensure data is bytes
            if isinstance(data, str):
                payload = data.encode("utf-8")
            else:
                payload = data
            self.sock.sendto(payload, self.last_addr)
            self.last_send_time = now
        except Exception as e:
            self.logger.error(f"Send Error: {e}")

    def stop(self):
        """
        Zatrzymuje menedżera UDP i zamyka gniazdo.
        Stops the UDP manager and closes the socket.
        """
        self.running = False
        if self.sock:
            self.sock.close()
        self.logger.info("UDP Manager stopped")
