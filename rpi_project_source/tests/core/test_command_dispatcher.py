"""PL: Testuje dekodowanie pakietów sterujących i reakcje failsafe.
EN: Tests control packet decoding and failsafe behavior.
"""

import struct
import unittest
from unittest.mock import MagicMock

from core.command_dispatcher import CommandDispatcher


class TestCommandDispatcher(unittest.TestCase):
    """PL: Testy selekcji i dekodowania poleceń przez dyspozytor.
    EN: Tests command selection and decoding by the dispatcher.
    """

    def setUp(self):
        """PL: Tworzy atrapę usługi i menedżera sprzętu dla testów.
        EN: Creates service and hardware manager mocks for the tests.
        """
        # Mock TelemetryWorker
        self.mock_worker = MagicMock()
        self.mock_worker.pca_armed = True
        self.mock_worker.current_mode = "MANUAL"
        self.mock_worker.link_established = False

        # Mock HardwareManager and Actuators
        self.mock_hw = MagicMock()
        self.mock_worker.hw_manager = self.mock_hw
        self.mock_hw.actuators.steering_range = (1000, 2000)
        self.mock_hw.actuators.throttle_range = (1000, 2000)

        self.dispatcher = CommandDispatcher(self.mock_worker)

    def _create_binary_packet(self, channels, tx_time=None):
        """PL: Buduje binarny pakiet CT z kanałami, czasem i sumą kontrolną.
        EN: Builds a CT binary packet with channels, timestamp, and checksum.
        """
        header = b"CT"
        num_ch = len(channels)

        if num_ch == 8:
            payload = struct.pack("<8H", *channels)
        elif num_ch == 16:
            payload = struct.pack("<16H", *channels)
        else:
            raise ValueError("Only 8 or 16 channels supported in helper")

        packet = header + payload

        if tx_time is not None:
            packet += struct.pack("<d", tx_time)

        # Add checksum (XOR of everything before it)
        checksum = 0
        for b in packet:
            checksum ^= b
        packet += struct.pack("B", checksum)

        return packet

    def test_handle_binary_control_8ch_success(self):
        """PL: Sprawdza dekodowanie ośmiu kanałów i znacznika czasu.
        EN: Checks decoding of eight channels and the packet timestamp.
        """
        # 8 channels, 1500 is center
        channels = [1500] * 8
        channels[0] = 1750  # Steering half right (0.5)
        channels[1] = 1250  # Throttle half back (-0.5)

        packet = self._create_binary_packet(channels, tx_time=123.456)

        self.dispatcher.on_data_received(packet)

        # Verify mapping
        controls = self.mock_worker.last_control_input["manual_controls"]
        self.assertAlmostEqual(controls["steering"], 0.5)
        self.assertAlmostEqual(controls["throttle"], -0.5)
        self.assertEqual(self.mock_worker.last_pc_timestamp, 123.456)
        self.assertTrue(self.mock_worker.link_established)

    def test_handle_binary_control_16ch_no_time(self):
        """PL: Sprawdza pakiet szesnastokanałowy bez znacznika czasu.
        EN: Checks a sixteen-channel packet without a timestamp.
        """
        channels = [1500] * 16
        packet = self._create_binary_packet(channels)

        self.mock_worker.last_pc_timestamp = 0.0
        self.dispatcher.on_data_received(packet)

        controls = self.mock_worker.last_control_input["manual_controls"]
        self.assertEqual(controls["steering"], 0.0)
        self.assertEqual(self.mock_worker.last_pc_timestamp, 0.0)

    def test_handle_binary_control_crc_error(self):
        """PL: Odrzuca pakiet z niezgodną sumą kontrolną.
        EN: Rejects a packet with an invalid checksum.
        """
        channels = [1500] * 8
        packet = bytearray(self._create_binary_packet(channels))

        # Corrupt packet
        packet[5] = (packet[5] + 1) % 256

        self.mock_worker.last_control_input = None
        self.dispatcher.on_data_received(bytes(packet))

        self.assertIsNone(self.mock_worker.last_control_input)

    def test_handle_binary_control_disarmed(self):
        """PL: Nie przekazuje sterowania, gdy kontroler jest rozbrojony.
        EN: Does not forward controls while the controller is disarmed.
        """
        self.mock_worker.pca_armed = False
        channels = [1500] * 8
        packet = self._create_binary_packet(channels)

        self.mock_worker.last_control_input = None
        self.dispatcher.on_data_received(packet)

        self.assertIsNone(self.mock_worker.last_control_input)

    def test_failsafe_recovery(self):
        """PL: Weryfikuje powrót do trybu manualnego po stabilnych pakietach.
        EN: Verifies recovery to manual mode after stable packets arrive.
        """
        self.mock_worker.current_mode = "FAILSAFE"
        channels = [1500] * 8
        packet = self._create_binary_packet(channels)

        # First 4 packets should maintain FAILSAFE (hysteresis requirement: 5 packets)
        for _ in range(4):
            self.dispatcher.on_data_received(packet)
            self.assertEqual(self.mock_worker.current_mode, "FAILSAFE")

        # 5th packet verifies link stability and transitions to MANUAL
        self.dispatcher.on_data_received(packet)
        self.assertEqual(self.mock_worker.current_mode, "MANUAL")


if __name__ == "__main__":
    unittest.main()
