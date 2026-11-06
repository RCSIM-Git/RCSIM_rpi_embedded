"""Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

PL: Obsługuje odbiornik GPS, komunikaty NMEA i opcjonalne poprawki NTRIP.
EN: Handles the GPS receiver, NMEA messages, and optional NTRIP corrections.
"""

import base64
import logging
import queue
import socket
import threading
import time
from typing import Any

# Import serial for type hinting, but handle its absence gracefully.
try:
    import serial
    from pynmeagps import NMEAParseError, NMEAReader

    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

    # Create a dummy serial class for type hinting to work
    class serial:
        """PL: Udostępnia zastępczy typ portu szeregowego bez pyserial.
        EN: Provides a fallback serial type when pyserial is unavailable.
        """

        Serial = object


class GPS_UART:
    """PL: Odczytuje pozycję GPS i obsługuje opcjonalne połączenie RTK/NTRIP.
    EN: Reads GPS position and supports an optional RTK/NTRIP connection.
    """

    RTK_QUALITY_MAP = {
        0: "Brak",
        1: "SPS",
        2: "DGPS",
        4: "RTK Fixed",
        5: "RTK Float",
    }

    def __init__(self, port: str, baudrate: int, ntrip_cfg: dict | None = None) -> None:
        """PL: Zapisuje parametry portu, konfigurację NTRIP i stan odczytów.
        EN: Stores port parameters, NTRIP configuration, and reading state.
        """
        self.logger = logging.getLogger(__name__)
        self.port = port
        self.baudrate = baudrate
        self.ntrip_cfg = ntrip_cfg or {}
        self.last_data = {
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "speed": 0.0,
            "course": 0.0,
            "fix": 0,
            "sats": 0,
            "rtk_status": "N/A",
            "h_accuracy": 99.0,
        }

        self.ntrip_client: GPS_UART.AdaptedNtripClient | None = None
        self.thread: threading.Thread | None = None
        self.parsed_queue = queue.Queue()

    def start(self) -> None:
        """PL: Uruchamia czytnik GPS albo klienta NTRIP w osobnym wątku.
        EN: Starts the GPS reader or NTRIP client in a separate thread.
        """
        if not HW_AVAILABLE:
            self.logger.error(
                "pyserial/pynmeagps libraries not found. GPS module is inactive."
            )
            self.last_data["rtk_status"] = "No HW"
            return

        if not self.ntrip_cfg.get("enabled", False):
            self.logger.warning(
                "NTRIP is disabled in configuration. GPS is running without RTK."
            )
            self.last_data["rtk_status"] = "NTRIP OFF"
            # Fallback to a simple reader thread if NTRIP is off
            # Użycie prostego wątku czytnika, jeśli NTRIP jest wyłączony
            self.thread = threading.Thread(
                target=self._simple_reader, daemon=True, name="SimpleGPSReaderThread"
            )
        else:
            self.last_data["rtk_status"] = "Init NTRIP"
            self.ntrip_client = self.AdaptedNtripClient(
                self.ntrip_cfg,
                self.port,
                self.baudrate,
                self.parsed_queue,
            )
            self.thread = threading.Thread(
                target=self.ntrip_client.run, daemon=True, name="NtripClientThread"
            )

        self.thread.start()
        self.logger.info("GPS thread started.")

    def _simple_reader(self) -> None:
        """PL: Czyta komunikaty NMEA z portu szeregowego bez NTRIP.
        EN: Reads NMEA messages from the serial port without NTRIP.
        """
        while True:
            try:
                with serial.Serial(self.port, self.baudrate, timeout=2) as stream:
                    nmr = NMEAReader(stream)
                    for raw, parsed in nmr:
                        if parsed:
                            self.parsed_queue.put(parsed)
            except serial.SerialException as e:
                self.logger.error(f"GPS serial port error: {e}. Retrying in 5s.")
                time.sleep(5)
            except Exception as e:
                self.logger.error(f"Unexpected error in simple_reader: {e}")
                time.sleep(5)

    def get_latest_data(self) -> dict[str, Any]:
        """PL: Przetwarza zakolejkowane komunikaty i zwraca bieżący stan GPS.
        EN: Processes queued messages and returns the current GPS state.
        """
        while not self.parsed_queue.empty():
            try:
                msg = self.parsed_queue.get_nowait()
                if not hasattr(msg, "msgID"):
                    continue

                if msg.msgID == "GGA":
                    # [PLAN-005] Zero-Jump Guard:
                    # Only update if lat/lon are non-zero (unless we really are at 0,0)
                    new_lat = msg.lat or 0.0
                    new_lon = msg.lon or 0.0

                    if abs(new_lat) > 1e-7 or abs(new_lon) > 1e-7:
                        self.last_data.update(
                            {
                                "lat": new_lat,
                                "lon": new_lon,
                                "alt": msg.alt or 0.0,
                                "sats": msg.numSV or 0,
                                "fix": msg.quality or 0,
                                "h_accuracy": msg.HDOP or 99.0,
                            }
                        )
                    else:
                        # Może mamy fix ale 0,0? Odrzucamy fix i ustawiamy status na 0 (brak)
                        self.logger.warning(
                            "GPS reported fix but coordinates are 0.0, 0.0. Discarding fix."
                        )
                        self.last_data.update(
                            {
                                "sats": msg.numSV or 0,
                                "fix": 0,  # 0 = Brak
                            }
                        )

                    self.last_data["rtk_status"] = self.RTK_QUALITY_MAP.get(
                        self.last_data["fix"], f"N/A ({self.last_data['fix']})"
                    )
                elif msg.msgID in ("RMC", "VTG"):
                    self.last_data["speed"] = (
                        getattr(msg, "sogk", self.last_data["speed"]) or 0.0
                    )
                    self.last_data["course"] = (
                        getattr(msg, "cogt", self.last_data["course"]) or 0.0
                    )
            except (queue.Empty, AttributeError, TypeError, ValueError) as e:
                self.logger.debug(f"Error processing NMEA message: {e}")
        return self.last_data

    def stop(self) -> None:
        """PL: Zatrzymuje klienta NTRIP i kończy pracę modułu GPS.
        EN: Stops the NTRIP client and shuts down the GPS module.
        """
        if self.ntrip_client:
            self.ntrip_client.stop()
        if self.thread and self.thread.is_alive():
            # Nie czekamy na dołączenie wątku, ponieważ może być zablokowany na IO
            # Don't wait for thread to join, as it might be blocked on IO
            pass
        self.logger.info("GPS module stopped.")

    class AdaptedNtripClient:
        """PL: Łączy strumień poprawek NTRIP z portem szeregowym GPS.
        EN: Connects the NTRIP correction stream to the GPS serial port.
        """

        def __init__(
            self,
            ntrip_config: dict,
            serial_port: str,
            baudrate: int,
            parsed_queue: queue.Queue,
        ) -> None:
            """PL: Zapisuje ustawienia serwera i przygotowuje stan połączenia.
            EN: Stores server settings and prepares the connection state.
            """
            self.logger = logging.getLogger("NtripClient")
            self.user_b64 = base64.b64encode(
                f"{ntrip_config.get('user', '')}:{ntrip_config.get('password', '')}".encode(
                    "utf-8"
                )
            ).decode("utf-8")
            self.caster, self.port = ntrip_config.get("host"), ntrip_config.get(
                "port", 2101
            )
            self.mountpoint = f"/{ntrip_config.get('mountpoint')}"
            self.serial_port_path = serial_port
            self.baudrate = baudrate
            self.parsed_queue = parsed_queue
            self._running = True
            self.threads: list[threading.Thread] = []

        def stop(self) -> None:
            """PL: Sygnalizuje pętli klienta NTRIP zakończenie pracy.
            EN: Signals the NTRIP client loop to stop running.
            """
            self._running = False

        def run(self) -> None:
            """PL: Utrzymuje połączenia NTRIP i szeregowe oraz obsługuje ponowienia.
            EN: Maintains NTRIP and serial connections and handles retries.
            """
            while self._running:
                stream: serial.Serial | None = None
                sock: socket.socket | None = None
                try:
                    if not self._check_hardware_available():
                        continue

                    stream = self._connect_serial()
                    sock = self._connect_ntrip_socket()
                    self._start_io_threads(sock, stream)

                except (
                    serial.SerialException,
                    socket.error,
                    ConnectionRefusedError,
                    IOError,
                ) as e:
                    self.logger.error(
                        f"NTRIP/Serial connection error: {e}. Restarting in 5s."
                    )
                finally:
                    self._cleanup_connections(sock, stream)
            self.logger.info("NTRIP client thread stopped.")

        def _check_hardware_available(self) -> bool:
            """PL: Sprawdza, czy biblioteki portu szeregowego są dostępne.
            EN: Checks whether the serial communication libraries are available.
            """
            if not HW_AVAILABLE:
                self.logger.warning(
                    "Cannot run NTRIP client, serial library not found."
                )
                time.sleep(5)
                return False
            return True

        def _connect_serial(self) -> serial.Serial:
            """PL: Otwiera port szeregowy i wysyła konfigurację do GPS.
            EN: Opens the serial port and sends configuration to the GPS.
            """
            stream = serial.Serial(self.serial_port_path, self.baudrate, timeout=1)
            self._configure_gps_module(stream)
            return stream

        def _connect_ntrip_socket(self) -> socket.socket:
            """PL: Otwiera gniazdo TCP i wykonuje handshake z casterem NTRIP.
            EN: Opens a TCP socket and performs a handshake with the NTRIP caster.
            """
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((self.caster, self.port))

            req = (
                f"GET {self.mountpoint} HTTP/1.1\r\nUser-Agent: NTRIP RCSIM/2.0\r\n"
                f"Authorization: Basic {self.user_b64}\r\n\r\n"
            ).encode("ascii")
            sock.sendall(req)

            if b"ICY 200 OK" not in sock.recv(4096):
                raise ConnectionRefusedError("NTRIP server rejected connection")

            self.logger.info("NTRIP connection successful.")
            return sock

        def _start_io_threads(self, sock: socket.socket, stream: serial.Serial) -> None:
            """PL: Uruchamia równoległy odczyt NMEA i przekazywanie danych RTCM.
            EN: Starts concurrent NMEA reading and RTCM forwarding.
            """
            self.logger.info("Starting I/O threads.")
            # Upewnij się, że _running jest True przed startem / Ensure _running is True before starting
            self._running = True

            t_rtcm = threading.Thread(
                target=self._rtcm_writer,
                args=(sock, stream),
                daemon=True,
                name="RTCMWriter",
            )
            t_nmea = threading.Thread(
                target=self._nmea_reader,
                args=(sock, stream),
                daemon=True,
                name="NMEAReader",
            )

            self.threads = [t_rtcm, t_nmea]
            for t in self.threads:
                t.start()

            # Czekaj aż którykolwiek wątek zakończy działanie
            # Wait for either thread to finish
            while self._running:
                if not t_rtcm.is_alive() or not t_nmea.is_alive():
                    self.logger.warning(
                        "One of the GPS I/O threads died. Stopping others."
                    )
                    break
                time.sleep(0.5)

            # Sygnalizuj obu zatrzymanie / Signal both to stop
            self._running = False
            for t in self.threads:
                t.join(timeout=2.0)

        def _cleanup_connections(
            self, sock: socket.socket | None, stream: serial.Serial | None
        ) -> None:
            """PL: Zamyka gniazdo i port szeregowy oraz czyści listę wątków.
            EN: Closes the socket and serial port and clears the thread list.
            """
            if sock:
                sock.close()
            if stream:
                stream.close()
            self.threads = []
            if self._running:
                time.sleep(5)

        def _configure_gps_module(self, stream: serial.Serial) -> None:
            """PL: Wysyła wymagane polecenia konfiguracyjne do odbiornika GPS.
            EN: Sends the required configuration commands to the GPS receiver.
            """
            if not HW_AVAILABLE:
                return
            self.logger.info("Configuring GPS module (LC29H)...")
            stream.reset_input_buffer()
            commands = [
                b"$PQCFGMSG,RTCM,0,0,0,0,0*4E\r\n",
                b"$PQTMGNSSRATE,5,1*1A\r\n",
                b"$PQTMGNSSMSG,GGA,1*2A\r\n",
                b"$PQTMGNSSMSG,RMC,1*21\r\n",
                b"$PQTMGNSSMSG,VTG,1*26\r\n",
                b"$PQTMGNSSMSG,GSA,0*3B\r\n",
                b"$PQTMGNSSMSG,GSV,0*3C\r\n",
                b"$PQCFGMSG,RTCM,1,1,0,0,0*4E\r\n",
            ]
            for cmd in commands:
                self.logger.debug(f"  -> Sending to GPS: {cmd.strip().decode()}")
                stream.write(cmd)
                time.sleep(0.2)
            self.logger.info("GPS module configuration finished.")

        def _rtcm_writer(self, sock: socket.socket, stream: serial.Serial) -> None:
            """PL: Przekazuje poprawki RTCM z serwera NTRIP do odbiornika GPS.
            EN: Forwards RTCM corrections from the NTRIP server to the GPS receiver.
            """
            self.logger.info("RTCM writer thread started.")
            while self._running:
                try:
                    # Ustaw mniejszy wewnętrzny timeout, aby umożliwić sprawdzenie self._running
                    # Set a smaller internal timeout for recv to allow checking self._running
                    sock.settimeout(2.0)
                    try:
                        rtcm_data = sock.recv(2048)
                    except socket.timeout:
                        continue  # Po prostu pętla i sprawdzenie self._running / Just loop back and check self._running

                    if not rtcm_data:
                        self.logger.warning("NTRIP stream closed by server.")
                        break
                    stream.write(rtcm_data)
                except (
                    socket.error,
                    serial.SerialException,
                ) as e:
                    # Błąd może być tymczasowy, loguj i zakończ wątek I/O
                    # Error might be temporary, log and break I/O thread
                    self.logger.error(f"Error in RTCM writer thread: {e}")
                    break

            self.logger.info("RTCM writer thread exiting.")

        def _nmea_reader(self, sock: socket.socket, stream: serial.Serial) -> None:
            """PL: Odczytuje NMEA z GPS i wysyła komunikaty GGA do NTRIP.
            EN: Reads NMEA from the GPS and sends GGA messages to NTRIP.
            """
            nmr = NMEAReader(stream)
            while self._running:
                try:
                    raw, parsed = nmr.read()
                    if raw:
                        if parsed and parsed.msgID == "GGA":
                            sock.sendall(raw)
                    if parsed:
                        self.parsed_queue.put(parsed)
                except (serial.SerialException, socket.error) as e:
                    self.logger.error(f"Error in NMEA reader thread: {e}")
                    break
                except (NMEAParseError, UnicodeDecodeError) as e:
                    self.logger.warning(f"NMEA parsing error: {e}")

            self.logger.info("NMEA reader thread exiting.")
