"""
Sterownik dla LiDAR LD08 (LD19).
Zoptymalizowany pod kątem wysokiej przepustowości (Zero-Lag Buffer).
Zaimplementowano poprawioną tabelę CRC8 (256 bajtów) i stabilny parser.
"""

import collections
import logging
import threading
import time
from typing import List, Optional, Tuple

import serial

# Stałe z dokumentacji / Constants from datasheet
PACKET_SIZE: int = 47
HEADER: int = 0x54
VERLEN: int = 0x2C

# Pełna tabela CRC8 (Polynomial 0x4D) - 256 wpisów
CRC_TABLE: list[int] = [
    0x00,
    0x4D,
    0x9A,
    0xD7,
    0x79,
    0x34,
    0xE3,
    0xAE,
    0xF2,
    0xBF,
    0x68,
    0x25,
    0x8B,
    0xC6,
    0x11,
    0x5C,
    0xA9,
    0xE4,
    0x33,
    0x7E,
    0xD0,
    0x9D,
    0x4A,
    0x07,
    0x5B,
    0x16,
    0xC1,
    0x8C,
    0x22,
    0x6F,
    0xB8,
    0xF5,
    0x1F,
    0x52,
    0x85,
    0xC8,
    0x66,
    0x2B,
    0xFC,
    0xB1,
    0xED,
    0xA0,
    0x77,
    0x3A,
    0x94,
    0xD9,
    0x0E,
    0x43,
    0xB6,
    0xFB,
    0x2C,
    0x61,
    0xCF,
    0x82,
    0x55,
    0x18,
    0x44,
    0x09,
    0xDE,
    0x93,
    0x3D,
    0x70,
    0xA7,
    0xEA,
    0x3E,
    0x73,
    0xA4,
    0xE9,
    0x47,
    0x0A,
    0xDD,
    0x90,
    0xCC,
    0x81,
    0x56,
    0x1B,
    0xB5,
    0xF8,
    0x2F,
    0x62,
    0x97,
    0xDA,
    0x0D,
    0x40,
    0xEE,
    0xA3,
    0x74,
    0x39,
    0x65,
    0x28,
    0xFF,
    0xB2,
    0x1C,
    0x51,
    0x86,
    0xCB,
    0x21,
    0x6C,
    0xBB,
    0xF6,
    0x58,
    0x15,
    0xC2,
    0x8F,
    0xD3,
    0x9E,
    0x49,
    0x04,
    0xAA,
    0xE7,
    0x30,
    0x7D,
    0x88,
    0xC5,
    0x12,
    0x5F,
    0xF1,
    0xBC,
    0x6B,
    0x26,
    0x7A,
    0x37,
    0xE0,
    0xAD,
    0x03,
    0x4E,
    0x99,
    0xD4,
    0x7C,
    0x31,
    0xE6,
    0xAB,
    0x05,
    0x48,
    0x9F,
    0xD2,
    0x8E,
    0xC3,
    0x14,
    0x59,
    0xF7,
    0xBA,
    0x6D,
    0x20,
    0xD5,
    0x98,
    0x4F,
    0x02,
    0xAC,
    0xE1,
    0x36,
    0x7B,
    0x27,
    0x6A,
    0xBD,
    0xF0,
    0x5E,
    0x13,
    0xC4,
    0x89,
    0x63,
    0x2E,
    0xF9,
    0xB4,
    0x1A,
    0x57,
    0x80,
    0xCD,
    0x91,
    0xDC,
    0x0B,
    0x46,
    0xE8,
    0xA5,
    0x72,
    0x3F,
    0xCA,
    0x87,
    0x50,
    0x1D,
    0xB3,
    0xFE,
    0x29,
    0x64,
    0x38,
    0x75,
    0xA2,
    0xEF,
    0x41,
    0x0C,
    0xDB,
    0x96,
    0x42,
    0x0F,
    0xD8,
    0x95,
    0x3B,
    0x76,
    0xA1,
    0xEC,
    0xB0,
    0xFD,
    0x2A,
    0x67,
    0xC9,
    0x84,
    0x53,
    0x1E,
    0xEB,
    0xA6,
    0x71,
    0x3C,
    0x92,
    0xDF,
    0x08,
    0x45,
    0x19,
    0x54,
    0x83,
    0xCE,
    0x60,
    0x2D,
    0xFA,
    0xB7,
    0x5D,
    0x10,
    0xC7,
    0x8A,
    0x24,
    0x69,
    0xBE,
    0xF3,
    0xAF,
    0xE2,
    0x35,
    0x78,
    0xD6,
    0x9B,
    0x4C,
    0x01,
    0xF4,
    0xB9,
    0x6E,
    0x23,
    0x8D,
    0xC0,
    0x17,
    0x5A,
    0x06,
    0x4B,
    0x9C,
    0xD1,
    0x7F,
    0x32,
    0xE5,
    0xA8,
]


def calc_crc8(data: bytes) -> int:
    """Oblicza CRC-8 na podstawie tabeli."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[crc ^ byte]
    return crc


class LD08Driver:
    """Sterownik LiDAR LD08 (Producer-Consumer Workflow)."""

    def __init__(self, port: str, baudrate: int = 230400) -> None:
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = False
        self.scan_buffer: List[Tuple[float, int]] = []
        self.full_scan: Optional[List[Tuple[float, int]]] = None
        self.lock = threading.Lock()
        self.last_start_angle = 0.0

        # Statystyki / Diagnostics
        self.pkt_total = 0
        self.pkt_crc_err = 0
        self.pkt_verlen_err = 0
        self.rotations = 0

        # Kolejka asynchroniczna / Async packet queue
        self.packet_queue: collections.deque[bytes] = collections.deque(maxlen=500)

    def start(self) -> None:
        """Uruchamia wątki odczytu i parsowania."""
        try:
            logging.info(f"LiDAR: Opening {self.port} at {self.baudrate}")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            self.is_running = True

            # Reader thread (Low level UART)
            threading.Thread(
                target=self._read_loop, daemon=True, name="LidarReader"
            ).start()
            # Parser thread (CPU heavy)
            threading.Thread(
                target=self._parse_loop, daemon=True, name="LidarParser"
            ).start()
            logging.info("LiDAR: Threads started.")
        except Exception as e:
            msg = f"LiDAR: Hardware disconnected or port {self.port} busy: {e}"
            logging.warning(msg)
            self.is_running = False

    def stop(self) -> None:
        """Zatrzymuje pracę sterownika."""
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _read_loop(self) -> None:
        """Wątek Reader: pobiera surowe bajty do kolejki pakietów."""
        buffer = bytearray()
        while self.is_running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                    while len(buffer) >= PACKET_SIZE:
                        if buffer[0] == HEADER and buffer[1] == VERLEN:
                            self.packet_queue.append(bytes(buffer[:PACKET_SIZE]))
                            del buffer[:PACKET_SIZE]
                        else:
                            # Szukaj nagłówka jeśli synchronizacja została utracona
                            idx = buffer.find(b"\x54\x2c", 1)
                            if idx != -1:
                                del buffer[:idx]
                            else:
                                buffer.clear()
                                break
                else:
                    time.sleep(0.001)
            except Exception as e:
                logging.debug(f"LiDAR Reader Error: {e}")
                time.sleep(0.1)

    def _parse_loop(self) -> None:
        """Wątek Parser: dekoduje pakiety z kolejki."""
        while self.is_running:
            try:
                if self.packet_queue:
                    packet = self.packet_queue.popleft()
                    self._parse_packet(packet)
                else:
                    time.sleep(0.002)
            except Exception as e:
                logging.debug(f"LiDAR Parser Error: {e}")
                time.sleep(0.01)

    def _parse_packet(self, packet: bytes) -> None:
        """Dekoduje ramkę danych (47 bajtów)."""
        self.pkt_total += 1

        # CRC Check
        received_crc = packet[-1]
        payload = packet[:-1]
        if calc_crc8(payload) != received_crc:
            self.pkt_crc_err += 1
            return

        # Data extraction
        # Speed: bytes 2,3 (deg/sec)
        # lid_speed = int.from_bytes(packet[2:4], "little")

        # Angles: start (4,5) and end (42,43)
        start_angle = int.from_bytes(packet[4:6], "little") / 100.0
        end_angle = int.from_bytes(packet[42:44], "little") / 100.0

        # Linear interpolation for 12 samples in packet
        angle_diff = (end_angle - start_angle) % 360.0
        step = angle_diff / 11.0

        with self.lock:
            # Check for new rotation
            if start_angle < self.last_start_angle - 10.0:
                if len(self.scan_buffer) > 50:
                    self.full_scan = self.scan_buffer.copy()
                    self.rotations += 1
                self.scan_buffer.clear()

            self.last_start_angle = start_angle

            # Safety clamp for buffer
            if len(self.scan_buffer) > 1500:
                self.scan_buffer.clear()

            # Process 12 distance & intensity samples
            for i in range(12):
                offset = 6 + i * 3
                dist = int.from_bytes(packet[offset : offset + 2], "little")
                # intensity = packet[offset + 2]

                angle = (start_angle + i * step) % 360.0
                if dist > 0:
                    self.scan_buffer.append((angle, dist))

    def read_scan(self, downsample: bool = False) -> Optional[List[Tuple[float, int]]]:
        """Pobiera skompilowany skan 360 stopni."""
        with self.lock:
            if not self.full_scan:
                return None
            scan_data = self.full_scan
            self.full_scan = None
            if downsample:
                return scan_data[::2]
            return scan_data
