from queue import Full
from unittest.mock import MagicMock

import pytest
from core.telemetry_sender import TelemetrySender


@pytest.fixture
def mock_services():
    """Tworzy mocki dla usług WebRTC i UDP."""
    return MagicMock(), MagicMock()


@pytest.fixture
def sender(mock_services):
    """Inicjalizuje TelemetrySender z mockowanymi usługami."""
    webrtc, udp = mock_services
    return TelemetrySender(webrtc, udp)


def test_send_packet_success(sender):
    """Test poprawnego dodania pakietu do kolejki."""
    packet = {"type": "test"}
    sender.send_packet(packet, "telemetry")

    assert not sender.queue.empty()
    queued_packet, label = sender.queue.get()
    assert queued_packet == packet
    assert label == "telemetry"


def test_send_packet_queue_full(sender):
    """Test odporności na pełną kolejkę (powinien po prostu zignorować pakiet)."""
    # Mockujemy metodę put_nowait, aby rzuciła wyjątek Full
    sender.queue.put_nowait = MagicMock(side_effect=Full)

    packet = {"type": "test_full"}
    # Wywołanie nie powinno rzucić wyjątku
    sender.send_packet(packet)

    sender.queue.put_nowait.assert_called_once()


def test_send_packet_generic_exception(sender):
    """Test odporności na nieoczekiwane wyjątki podczas zapisu do kolejki."""
    sender.queue.put_nowait = MagicMock(side_effect=Exception("Unexpected Error"))

    packet = {"type": "test_err"}
    # Wywołanie nie powinno rzucić wyjątku mimo błędu wewnątrz
    sender.send_packet(packet)

    sender.queue.put_nowait.assert_called_once()


def test_broadcast_both_services(sender, mock_services):
    """Weryfikacja rozsyłania danych do obu usług (WebRTC i UDP)."""
    webrtc, udp = mock_services
    data_str = '{"type": "telemetry"}'

    sender._broadcast(data_str, "telemetry")

    webrtc.send_data.assert_called_once_with(data_str, "telemetry")
    udp.send_data.assert_called_once_with(data_str)


def test_broadcast_webrtc_failure_does_not_block_udp(sender, mock_services):
    """Błąd w WebRTC nie powinien blokować wysyłki przez UDP."""
    webrtc, udp = mock_services
    webrtc.send_data.side_effect = Exception("WebRTC Error")
    data_str = '{"type": "telemetry"}'

    sender._broadcast(data_str, "telemetry")

    webrtc.send_data.assert_called_once()
    udp.send_data.assert_called_once_with(data_str)
