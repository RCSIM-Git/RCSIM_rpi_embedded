"""PL: Archiwalny parser ramek CRSF zachowany dla kompatybilności wstecznej.

Odbiera kanały RC i statystyki łącza z UART oraz może nadawać wybrane dane
telemetryczne. Protokół CRSF/FBW został zastąpiony przez MAVLink.

EN: Archived CRSF frame parser retained for backward compatibility.

It receives RC channels and link statistics over UART and can transmit selected
telemetry data. The CRSF/FBW protocol has been superseded by MAVLink.

Parsuje binarne ramki CRSF z urządzeń Crossfire/ExpressLRS przez port szeregowy.
Specyfikacja RC_CHANNELS_PACKED:
Sync(0xC8) | Length(24) | Type(0x16) | Payload(22 bytes: 16 przyległych 11-bitówek) | CRC8(1 byte)
Wymaga `pyserial`.
"""

import logging
import queue
import struct
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import serial


class CRSFParser:
    """PL: Odczytuje i dekoduje ramki CRSF z portu UART.

    Klasa jest przestarzała i zachowana wyłącznie dla kompatybilności.

    EN: Reads and decodes CRSF frames from a UART port.

    This class is deprecated and retained for compatibility only.
    """

    SYNC_BYTE = 0xC8
    TYPE_RC_CHANNELS_PACKED = 0x16
    TYPE_LINK_STATISTICS = 0x14
    MAX_PACKET_SIZE = 64
    BAUDRATE = 420000

    def __init__(
        self,
        port: str = "/dev/ttyAMA1",
        baudrate: int = BAUDRATE,
        logger: Optional[logging.Logger] = None,
    ):
        """PL: Przygotowuje parser i bufory dla wskazanego portu szeregowego.

        Args:
            port: Nazwa portu UART.
            baudrate: Szybkość transmisji portu.
            logger: Opcjonalny logger; domyślnie używany jest logger modułu.

        EN: Prepares the parser and buffers for the specified serial port.

        Args:
            port: UART device name.
            baudrate: Serial port baud rate.
            logger: Optional logger; the module logger is used by default.
        """
        self.port = port
        self.baudrate = baudrate
        self.logger = logger or logging.getLogger(__name__)

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._serial: Optional[serial.Serial] = None

        # Aktualne stany odebrane
        self.channels: List[int] = [1500] * 16  # zmapowane już na PWM (1000 - 2000)
        self.link_statistics = {
            "rssi_1": 0,
            "rssi_2": 0,
            "link_quality": 0,
            "snr": 0,
            "active_antenna": 0,
        }
        self.last_frame_time: float = 0.0

        # Telemetria do wysłania
        self.tx_queue: queue.Queue = queue.Queue()
        self._write_thread: Optional[threading.Thread] = None

        # Callback po poprawnej ramce
        self.on_channels_updated: Optional[Callable[[List[int]], None]] = None
        self.on_link_updated: Optional[Callable[[dict], None]] = None

        self._crc8_table = self._init_crc8_table()

    def _init_crc8_table(self) -> List[int]:
        """PL: Buduje tablicę CRC-8 z wielomianem 0xD5.

        Returns:
            256-elementową tablicę wartości CRC.

        EN: Builds a CRC-8 lookup table using polynomial 0xD5.

        Returns:
            A 256-entry table of CRC values.
        """
        poly = 0xD5
        table = [0] * 256
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
            table[i] = crc & 0xFF
        return table

    def _calc_crc8(self, data: bytes) -> int:
        """PL: Oblicza CRC-8 dla bajtów przy użyciu tablicy parsera.

        Args:
            data: Bajty objęte kontrolą CRC.

        Returns:
            Jednobajtową wartość CRC.

        EN: Calculates CRC-8 for the bytes using the parser lookup table.

        Args:
            data: Bytes covered by the CRC calculation.

        Returns:
            The one-byte CRC value.
        """
        crc = 0
        for byte in data:
            crc = self._crc8_table[crc ^ byte]
        return crc

    def start(self) -> None:
        """PL: Otwiera UART i uruchamia wątki odbioru oraz nadawania.

        EN: Opens the UART and starts the reader and writer threads.
        """
        if self._running:
            return

        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self._running = True
            self._thread = threading.Thread(
                target=self._read_loop, daemon=True, name="CRSF_Reader"
            )
            self._thread.start()
            self._write_thread = threading.Thread(
                target=self._write_loop, daemon=True, name="CRSF_Writer"
            )
            self._write_thread.start()
            self.logger.info(
                f"CRSFParser uruchomiony na porcie {self.port} @ {self.baudrate} baud (Full-Duplex)."
            )
        except serial.SerialException as e:
            self.logger.error(f"Nie udało się otworzyć portu {self.port}: {e}")

    def stop(self) -> None:
        """PL: Zatrzymuje wątki parsera i zamyka otwarty port szeregowy.

        EN: Stops the parser threads and closes the serial port if it is open.
        """
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._write_thread:
            self._write_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.logger.info("CRSFParser zatrzymany.")

    def _read_loop(self) -> None:
        """PL: Odbiera bajty UART, sprawdza ramki i przekazuje je do parsera.

        EN: Receives UART bytes, validates frames, and passes them to the parser.
        """
        buffer = bytearray()

        # Oczekujemy na Sync Byte
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    time.sleep(0.1)
                    continue

                waiting = self._serial.in_waiting
                if waiting > 0:
                    new_data = self._serial.read(min(waiting, 128))
                    if new_data:
                        buffer.extend(new_data)

                # Zabezpieczenie przed przepełnieniem (np. nieprawidłowe baudrate)
                if len(buffer) > self.MAX_PACKET_SIZE * 3:
                    buffer = buffer[-self.MAX_PACKET_SIZE :]

                # Próba znalezienia kompletnego pakiety
                while len(buffer) >= 4:  # min(Sync, Length, Type, CRC)
                    # Szukamy bajtu synchronizacji
                    try:
                        sync_idx = buffer.index(self.SYNC_BYTE)
                    except ValueError:
                        buffer.clear()
                        break

                    # Zrzut śmieci
                    if sync_idx > 0:
                        del buffer[:sync_idx]

                    if len(buffer) < 2:
                        break  # Czekamy na bajt długości

                    length = buffer[1]
                    # Walidacja limitu specyfikacji
                    if length < 2 or length > self.MAX_PACKET_SIZE:
                        # Wadliwe parsowanie - utknęliśmy np. na losowym bajcie rónym 0xC8.
                        # Wywalam ten bajt by szukać dalej.
                        del buffer[0:1]
                        continue

                    total_packet_size = length + 2  # (Sync + Length) + length

                    if len(buffer) < total_packet_size:
                        break  # Oczekujemy na resztę ramki (UART w toku)

                    # Sprawdzamy sumę kontrolną
                    packet = buffer[:total_packet_size]
                    del buffer[:total_packet_size]

                    payload_plus_type = packet[2:-1]  # typ to packet[2]
                    expected_crc = packet[-1]
                    calculated_crc = self._calc_crc8(payload_plus_type)

                    if expected_crc != calculated_crc:
                        # checksum error
                        self.logger.debug(
                            f"CRSF CRC Error. Oczekiwano: {expected_crc:02X}, Zdekodowano: {calculated_crc:02X}"
                        )
                        continue

                    # Poprawna ramka - dystrybuujemy do logiki systemu
                    self._parse_packet(packet)

            except serial.SerialException as e:
                self.logger.error(f"CRSF Utracono serial: {e}")
                time.sleep(
                    1
                )  # Chwilowy relaks przed ponowna proba w rzucanym Exception

            except Exception as e:
                self.logger.error(f"Błąd krytyczny w pętli parsera: {e}")
                time.sleep(1)

    def _parse_packet(self, packet: bytes) -> None:
        """PL: Przekazuje payload do dekodera właściwego typu ramki CRSF.

        Args:
            packet: Kompletna ramka CRSF z poprawnym CRC.

        EN: Dispatches the payload to the decoder for its CRSF frame type.

        Args:
            packet: Complete CRSF frame with a valid CRC.
        """
        packet_type = packet[2]
        payload = packet[3:-1]

        if packet_type == self.TYPE_RC_CHANNELS_PACKED:
            if len(payload) == 22:
                self._decode_rc_channels(payload)
                self.last_frame_time = time.monotonic()
                if self.on_channels_updated:
                    self.on_channels_updated(self.channels.copy())

        elif packet_type == self.TYPE_LINK_STATISTICS:
            if len(payload) == 10:
                self._decode_link_statistics(payload)
                if self.on_link_updated:
                    self.on_link_updated(self.link_statistics.copy())

    def _decode_rc_channels(self, payload: bytes) -> None:
        """PL: Dekoduje 16 kanałów 11-bitowych z 22-bajtowego payloadu CRSF.

        Przelicza wartości kanałów na mikrosekundy PWM i ogranicza ich zakres.

        Args:
            payload: Payload ramki RC_CHANNELS_PACKED.

        EN: Decodes 16 11-bit channels from a 22-byte CRSF payload.

        Converts channel values to PWM microseconds and clamps their range.

        Args:
            payload: Payload of an RC_CHANNELS_PACKED frame.
        """
        # Dla wygody zamieniam bajty na 1 potężny 176-bitowy integer
        # (little-endian z racji na specyfikację, ale po ułożeniu trzeba ciąć prawidłowo).
        # CRSF specyfikacja podaje, że pakiety składane są per 11 bitów zachodzące przez LSB/MSB.

        val = int.from_bytes(payload, byteorder="little")
        for idx in range(16):
            # Maskujemy 11 dolnych bitów z aktualnie ułożonej pętli.
            raw_val = val & 0x7FF
            val >>= 11

            # Konwersja z CRSF (172..1811) do zakresu PWM (1000..2000).
            # Wzorzec: pwm = (raw_val - 992) * (5/8) + 1500. Bardziej surowo: (raw_val * 1000) // 1639 -> ok ~ .
            # Skutecznie w Betaflight 1500 to 992.
            # Wzór zgodny ze standardem (EdgeTX): (crsf_val * 1024 / 1639) + 881. Możemy mapować na sztywno.

            # Odtworzenie na bazie interpolacji min/max (CRSF 191(988us) - 1792(2012us), środek 992(1500us))
            # pwm = (raw_val * 10) // 16 + 880 (Przybliżenie poprawne)

            pwm = int((raw_val - 992) * (500.0 / 800.0) + 1500)

            # Bezpieczny clamp:
            pwm = max(980, min(2020, pwm))
            self.channels[idx] = pwm

    def _decode_link_statistics(self, payload: bytes) -> None:
        """PL: Odczytuje RSSI, jakość łącza, SNR i aktywną antenę z payloadu.

        Args:
            payload: Payload ramki statystyk łącza CRSF.

        EN: Reads RSSI, link quality, SNR, and active antenna from the payload.

        Args:
            payload: Payload of a CRSF link statistics frame.
        """
        r1 = payload[0]
        r2 = payload[1]
        self.link_statistics["rssi_1"] = (r1 - 256) if r1 > 127 else -r1
        self.link_statistics["rssi_2"] = (r2 - 256) if r2 > 127 else -r2
        self.link_statistics["link_quality"] = payload[2]
        self.link_statistics["snr"] = int.from_bytes(
            payload[3:4], byteorder="little", signed=True
        )
        self.link_statistics["active_antenna"] = payload[4]

    # --- NADAWANIE TELEMETRII ---

    def queue_telemetry(self, t_type: str, data: Dict[str, Any]) -> None:
        """PL: Dodaje obsługiwane dane telemetryczne do kolejki nadawczej.

        Args:
            t_type: Typ telemetrii: battery, gps lub attitude.
            data: Wartości, które pętla nadawcza zakoduje do ramki.

        EN: Adds supported telemetry data to the transmit queue.

        Args:
            t_type: Telemetry type: battery, gps, or attitude.
            data: Values for the writer loop to encode into a frame.
        """
        self.tx_queue.put((t_type, data))

    def _write_loop(self) -> None:
        """PL: Koduje obsługiwane dane telemetrii i zapisuje ramki do UART.

        EN: Encodes supported telemetry data and writes frames to the UART.
        """
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    time.sleep(0.1)
                    continue

                t_type, data = self.tx_queue.get(timeout=0.1)

                payload = None
                frame_id = 0x00

                if t_type == "battery":
                    frame_id = 0x08
                    # voltage (0.1V), current (0.1A), capacity (mAh), percent
                    v = int(data.get("voltage", 0) * 10)
                    c = int(data.get("current", 0) * 10)
                    cap = int(data.get("capacity_drawn", 0))
                    pct = int(data.get("percent", 0))

                    # 10 bytes: voltage (2B), current (2B), capacity (3B), percent (1B), 2B unused/padding
                    # Struct formatting: >HH (4 bytes) + capacity (3 bytes) + percent (1 byte)
                    payload = struct.pack(
                        ">HHBBB",
                        v,
                        c,
                        (cap >> 16) & 0xFF,
                        (cap >> 8) & 0xFF,
                        cap & 0xFF
                    )
                    payload += bytes([pct & 0xFF])

                elif t_type == "gps":
                    frame_id = 0x02
                    # lat (1e7), lon (1e7), speed (0.1km/h), heading (0.01deg), alt (m + 1000), sats
                    lat = int(data.get("lat", 0) * 1e7)
                    lon = int(data.get("lon", 0) * 1e7)
                    spd = int(data.get("speed", 0) * 10)
                    hdg = int(data.get("heading", 0) * 100)
                    alt = int(data.get("altitude", 0) + 1000)
                    sats = int(data.get("satellites", 0))

                    payload = struct.pack(">iiHHHB", lat, lon, spd, hdg, alt, sats & 0xFF)

                elif t_type == "attitude":
                    frame_id = 0x1E
                    # pitch (10000 rad), roll, yaw
                    p = int(data.get("pitch", 0) * 10000)
                    r = int(data.get("roll", 0) * 10000)
                    y = int(data.get("yaw", 0) * 10000)

                    payload = struct.pack(">hhh", p, r, y)

                if payload:
                    length = len(payload) + 2
                    frame = bytearray([self.SYNC_BYTE, length, frame_id])
                    frame.extend(payload)
                    crc = self._calc_crc8(bytes([frame_id]) + payload)
                    frame.append(crc)

                    self._serial.write(frame)

            except queue.Empty:
                pass
            except serial.SerialException as e:
                self.logger.error(f"TX Serial Error: {e}")
                time.sleep(1)
            except Exception as e:
                self.logger.error(f"CRSF TX Telemetry Error: {e}")
                time.sleep(0.5)
