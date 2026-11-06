# -*- coding: utf-8 -*-
"""Copyright (c) 2026 RCSIM / Mateusz Buzek. All rights reserved.

PL: Testuje kolejność pakietów sterujących i priorytet łącza ELRS.
EN: Tests control packet ordering and ELRS link priority.
"""

import struct
import unittest
from unittest.mock import MagicMock
import json

from core.command_dispatcher import CommandDispatcher


class TestRPiPacketOrder(unittest.TestCase):
    """PL: Testy porządku chronologicznego pakietów RPi.
    EN: Tests chronological ordering of Raspberry Pi packets.
    """

    def setUp(self):
        """PL: Przygotowuje atrapę workera, sprzętu i dyspozytora.
        EN: Prepares worker and hardware mocks and a command dispatcher.
        """
        # Mock TelemetryWorker
        self.mock_worker = MagicMock()
        self.mock_worker.pca_armed = True
        self.mock_worker.current_mode = "MANUAL"
        self.mock_worker.comm_mode = "HYBRID"
        self.mock_worker.elrs_link_established = False
        self.mock_worker.last_pc_timestamp = 0.0

        # Mock HardwareManager i Actuators
        self.mock_hw = MagicMock()
        self.mock_worker.hw_manager = self.mock_hw
        self.mock_hw.actuators.steering_range = (1000, 2000)
        self.mock_hw.actuators.throttle_range = (1000, 2000)

        # Inicjalizacja dyspozytora komend
        self.dispatcher = CommandDispatcher(self.mock_worker)

    def _create_binary_packet(self, channels, tx_time=None):
        """PL: Buduje pakiet CT z kanałami, znacznikiem czasu i sumą XOR.
        EN: Builds a CT packet with channels, timestamp, and XOR checksum.
        """
        header = b"CT"
        num_ch = len(channels)

        if num_ch == 8:
            payload = struct.pack("<8H", *channels)
        elif num_ch == 16:
            payload = struct.pack("<16H", *channels)
        else:
            raise ValueError("Wspierane są tylko 8 lub 16 kanałów w pomocniku")

        packet = header + payload

        if tx_time is not None:
            packet += struct.pack("<d", tx_time)

        # Oblicz sumę kontrolną XOR
        checksum = 0
        for b in packet:
            checksum ^= b
        packet += struct.pack("B", checksum)

        return packet

    def test_binary_packet_monotonicity(self):
        """PL: Sprawdza akceptację rosnących i odrzucenie opóźnionych pakietów CT.
        EN: Checks acceptance of newer and rejection of out-of-order CT packets.
        """
        channels = [1500] * 8

        # 1. Pierwszy pakiet (czas 1000.0) -> sukces
        packet_1 = self._create_binary_packet(channels, tx_time=1000.0)
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(packet_1)
        
        self.assertIsNotNone(self.mock_worker.last_control_input)
        self.assertEqual(self.dispatcher.last_processed_tx_time, 1000.0)
        self.assertEqual(self.mock_worker.last_pc_timestamp, 1000.0)

        # 2. Pakiet starszy / opóźniony (czas 999.0) -> odrzucenie
        packet_2 = self._create_binary_packet(channels, tx_time=999.0)
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(packet_2)
        
        # Sterowanie nie powinno się zapisać, a znacznik czasu nie ulec zmianie
        self.assertIsNone(self.mock_worker.last_control_input)
        self.assertEqual(self.dispatcher.last_processed_tx_time, 1000.0)

        # 3. Kolejny poprawny pakiet (czas 1001.0) -> sukces
        packet_3 = self._create_binary_packet(channels, tx_time=1001.0)
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(packet_3)
        
        self.assertIsNotNone(self.mock_worker.last_control_input)
        self.assertEqual(self.dispatcher.last_processed_tx_time, 1001.0)
        self.assertEqual(self.mock_worker.last_pc_timestamp, 1001.0)

    def test_json_packet_monotonicity(self):
        """PL: Sprawdza kolejność znaczników czasu w pakietach JSON sterowania.
        EN: Checks timestamp ordering in JSON control packets.
        """
        # Ustawienie początkowego czasu przetworzenia
        self.dispatcher.last_processed_tx_time = 1500.0

        # 1. Prawidłowy JSON (czas 2000.0) -> sukces
        json_1 = {
            "type": "control",
            "channels": [1500, 1500],
            "t": 2000.0
        }
        self.mock_worker.last_control_input = None
        
        # Kodujemy do postaci bajtowej tak jak w sieci WebRTC/UDP
        self.dispatcher.on_data_received(json.dumps(json_1).encode('utf-8'))
        
        self.assertIsNotNone(self.mock_worker.last_control_input)
        self.assertEqual(self.dispatcher.last_processed_tx_time, 2000.0)
        self.assertEqual(self.mock_worker.last_pc_timestamp, 2000.0)

        # 2. Opóźniony JSON (czas 1999.0) -> odrzucenie
        json_2 = {
            "type": "control",
            "channels": [1500, 1500],
            "t": 1999.0
        }
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(json.dumps(json_2).encode('utf-8'))
        
        self.assertIsNone(self.mock_worker.last_control_input)
        self.assertEqual(self.dispatcher.last_processed_tx_time, 2000.0)

        # 3. Kolejny prawidłowy JSON (czas 2001.0) -> sukces
        json_3 = {
            "type": "control",
            "channels": [1500, 1500],
            "t": 2001.0
        }
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(json.dumps(json_3).encode('utf-8'))
        
        self.assertIsNotNone(self.mock_worker.last_control_input)
        self.assertEqual(self.dispatcher.last_processed_tx_time, 2001.0)
        self.assertEqual(self.mock_worker.last_pc_timestamp, 2001.0)

    def test_elrs_dominance_ignores_network_packets(self):
        """PL: Sprawdza odrzucanie pakietów sieciowych przy aktywnym ELRS.
        EN: Checks that network packets are rejected when ELRS is active.
        """
        self.mock_worker.elrs_link_established = True
        channels = [1500] * 8

        # Próba wstrzyknięcia binarnego CT
        packet = self._create_binary_packet(channels, tx_time=3000.0)
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(packet)
        
        # Sterowanie nie powinno się zapisać (odrzucone ze względu na ELRS)
        self.assertIsNone(self.mock_worker.last_control_input)

        # Próba wstrzyknięcia JSON
        json_packet = {
            "type": "control",
            "channels": [1500, 1500],
            "t": 4000.0
        }
        self.mock_worker.last_control_input = None
        
        self.dispatcher.on_data_received(json.dumps(json_packet).encode('utf-8'))
        
        # Sterowanie nie powinno się zapisać
        self.assertIsNone(self.mock_worker.last_control_input)


if __name__ == "__main__":
    unittest.main()
