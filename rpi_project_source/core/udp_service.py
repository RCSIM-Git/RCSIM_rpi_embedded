"""PL: Obsługuje odbieranie i wysyłanie datagramów UDP w trybie legacy.

EN: Handles UDP datagram reception and transmission in legacy mode.

Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""

import logging
import socket
import threading
from typing import Callable


class UDPService:
    """PL: Odbiera datagramy sterujące i wysyła dane UDP do klienta.

    Usługa zapamiętuje adres nadawcy ostatniego odebranego datagramu jako cel
    wysyłania, chyba że podano cel początkowy.

    EN: Receives control datagrams and sends UDP data to a client.

    The service remembers the sender address of the latest datagram as the
    destination for outgoing data, unless an initial destination was provided.
    """

    def __init__(
        self,
        port: int,
        on_data_received: Callable[[str], None],
        target_ip: str | None = None,
        target_port: int | None = None,
    ):
        """PL: Tworzy i wiąże gniazdo UDP oraz ustawia callback odbioru.

        Args:
            port: Port lokalny, na którym usługa będzie nasłuchiwać.
            on_data_received: Funkcja wywoływana po odebraniu datagramu.
            target_ip: Opcjonalny początkowy adres docelowy.
            target_port: Opcjonalny początkowy port docelowy.

        EN: Creates and binds a UDP socket and configures the receive callback.

        Args:
            port: Local port on which the service listens.
            on_data_received: Function called when a datagram is received.
            target_ip: Optional initial destination address.
            target_port: Optional initial destination port.
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
        """PL: Uruchamia wątek odbierający datagramy, jeśli nie działa.

        EN: Starts the datagram-receiving thread if it is not already running.
        """
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.logger.info(f"UDP Service started on port {self.port}")

    def stop(self):
        """PL: Zamyka gniazdo i czeka krótko na zakończenie wątku odbiorczego.

        EN: Closes the socket and briefly waits for the receive thread to stop.
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
        """PL: Wysyła dane do ustawionego lub ostatnio zapamiętanego adresu.

        Args:
            data: Tekst lub bajty datagramu do wysłania.

        EN: Sends data to the configured or most recently remembered address.

        Args:
            data: Text or bytes to send as a datagram.
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
        """PL: Odbiera datagramy, zapamiętuje adres nadawcy i wywołuje callback.

        EN: Receives datagrams, remembers the sender address, and calls the callback.
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
