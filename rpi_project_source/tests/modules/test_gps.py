from unittest.mock import MagicMock, patch

import pytest
from modules.gps import GPS_UART


class TestGPSUART:

    @pytest.fixture
    def gps_uart(self):
        """Fixture providing a GPS_UART instance with mocked dependencies."""
        with patch("modules.gps.HW_AVAILABLE", True):
            return GPS_UART(port="COM1", baudrate=9600)

    def test_rtk_quality_map_constant(self):
        """Verify the RTK_QUALITY_MAP constant exists and has correct values."""
        assert hasattr(GPS_UART, "RTK_QUALITY_MAP")
        assert GPS_UART.RTK_QUALITY_MAP[4] == "RTK Fixed"
        assert GPS_UART.RTK_QUALITY_MAP[5] == "RTK Float"
        assert GPS_UART.RTK_QUALITY_MAP[0] == "Brak"

    def test_get_latest_data_updates_rtk_status(self, gps_uart):
        """Verify get_latest_data correctly maps 'quality' to 'rtk_status'."""
        # Mock a GGA message
        mock_msg = MagicMock()
        mock_msg.msgID = "GGA"
        mock_msg.lat = 50.0
        mock_msg.lon = 20.0
        mock_msg.alt = 100.0
        mock_msg.numSV = 10
        mock_msg.HDOP = 0.5

        # Test Case 1: RTK Fixed (Quality 4)
        mock_msg.quality = 4
        gps_uart.parsed_queue.put(mock_msg)

        data = gps_uart.get_latest_data()
        assert data["rtk_status"] == "RTK Fixed"
        assert data["fix"] == 4

        # Test Case 2: RTK Float (Quality 5)
        mock_msg.quality = 5
        gps_uart.parsed_queue.put(mock_msg)

        data = gps_uart.get_latest_data()
        assert data["rtk_status"] == "RTK Float"

        # Test Case 3: Unknown Quality (Quality 99)
        mock_msg.quality = 99
        gps_uart.parsed_queue.put(mock_msg)

        data = gps_uart.get_latest_data()
        assert data["rtk_status"] == "N/A (99)"

    def test_get_latest_data_handles_empty_queue(self, gps_uart):
        """Verify get_latest_data returns last_data when queue is empty."""
        gps_uart.last_data["fix"] = 1
        gps_uart.last_data["rtk_status"] = "SPS"

        data = gps_uart.get_latest_data()
        assert data["fix"] == 1
        assert data["rtk_status"] == "SPS"
