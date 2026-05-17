import socket
import unittest
from unittest.mock import MagicMock, patch

from modules.udp_manager import UDPManager


class TestUDPManager(unittest.TestCase):
    def setUp(self):
        self.mock_callback = MagicMock()
        # Patch socket.socket and bind to avoid real network calls
        with patch("socket.socket") as mock_sock_cls:
            self.mock_socket = MagicMock()
            mock_sock_cls.return_value = self.mock_socket
            self.manager = UDPManager(port=9091, on_data_received=self.mock_callback)

    def test_initialization(self):
        self.assertEqual(self.manager.port, 9091)
        self.assertEqual(self.manager.callback, self.mock_callback)
        self.mock_socket.bind.assert_called_with(("0.0.0.0", 9091))

    def test_send_data_no_client(self):
        # Should not raise error or call sendto if no client has connected yet
        self.manager.send_data("test data")
        self.mock_socket.sendto.assert_not_called()

    def test_send_data_with_client(self):
        self.manager.last_addr = ("127.0.0.1", 12345)

        # Test string data
        self.manager.send_data("hello")
        self.mock_socket.sendto.assert_called_with(b"hello", ("127.0.0.1", 12345))

        # Test bytes data
        self.mock_socket.sendto.reset_mock()
        self.manager.send_data(b"binary")
        self.mock_socket.sendto.assert_called_with(b"binary", ("127.0.0.1", 12345))

    def test_receive_loop_logic(self):
        # Mock recvfrom to return data then raise timeout to exit (or we stop it manually)
        addr = ("192.168.1.100", 54321)
        test_msg = "command_json"

        # We simulate the loop by calling _loop once and making it exit
        self.mock_socket.recvfrom.side_effect = [
            (test_msg.encode("utf-8"), addr),
            socket.timeout,
        ]

        # Set running to false after the first successful receive to break the loop
        # But wait, the code uses a while loop. We can use side_effect to change self.running.
        def stop_running(*args, **kwargs):
            self.manager.running = False
            return (test_msg.encode("utf-8"), addr)

        self.mock_socket.recvfrom.side_effect = stop_running

        self.manager.running = True
        self.manager._loop()

        self.assertEqual(self.manager.last_addr, addr)
        self.mock_callback.assert_called_with(test_msg)

    def test_receive_decode_error(self):
        # Test non-utf8 data
        addr = ("1.1.1.1", 80)
        bad_data = b"\xff\xfe\xfd"

        def stop_running(*args, **kwargs):
            self.manager.running = False
            return (bad_data, addr)

        self.mock_socket.recvfrom.side_effect = stop_running
        self.manager.running = True

        with self.assertLogs("UDPManager", level="WARNING") as cm:
            self.manager._loop()
            self.assertIn("Received non-UTF8 data", cm.output[0])

        self.assertEqual(self.manager.last_addr, addr)
        self.mock_callback.assert_not_called()


if __name__ == "__main__":
    unittest.main()
