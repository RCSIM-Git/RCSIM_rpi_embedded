"""PL: Testuje kolejkowanie pakietów i wysyłanie telemetrii przez usługi.
EN: Tests packet queuing and telemetry broadcasting through services.
"""

from queue import Full
from unittest.mock import MagicMock

import pytest
from core.telemetry_sender import TelemetrySender


@pytest.fixture
def mock_services():
    """PL: Tworzy atrapy usług WebRTC i UDP.
    EN: Creates mocks for the WebRTC and UDP services.
    """
    return MagicMock(), MagicMock()


@pytest.fixture
def sender(mock_services):
    """PL: Tworzy TelemetrySender z atrapami transportów.
    EN: Creates a TelemetrySender with mocked transports.
    """
    webrtc, udp = mock_services
    return TelemetrySender(webrtc, udp)


def test_send_packet_success(sender):
    """PL: Sprawdza dodanie pakietu i etykiety do kolejki.
    EN: Checks that a packet and its label are added to the queue.
    """
    packet = {"type": "test"}
    sender.send_packet(packet, "telemetry")

    assert not sender.queue.empty()
    queued_packet, label = sender.queue.get()
    assert queued_packet == packet
    assert label == "telemetry"


def test_send_packet_queue_full(sender):
    """PL: Sprawdza bezpieczne pominięcie pakietu przy pełnej kolejce.
    EN: Checks that a packet is safely skipped when the queue is full.
    """
    # Mockujemy metodę put_nowait, aby rzuciła wyjątek Full
    sender.queue.put_nowait = MagicMock(side_effect=Full)

    packet = {"type": "test_full"}
    # Wywołanie nie powinno rzucić wyjątku
    sender.send_packet(packet)

    sender.queue.put_nowait.assert_called_once()


def test_send_packet_generic_exception(sender):
    """PL: Sprawdza obsługę nieoczekiwanego błędu dodania pakietu.
    EN: Checks handling of an unexpected error while queuing a packet.
    """
    sender.queue.put_nowait = MagicMock(side_effect=Exception("Unexpected Error"))

    packet = {"type": "test_err"}
    # Wywołanie nie powinno rzucić wyjątku mimo błędu wewnątrz
    sender.send_packet(packet)

    sender.queue.put_nowait.assert_called_once()


def test_broadcast_both_services(sender, mock_services):
    """PL: Sprawdza wysyłkę telemetrii do transportów WebRTC i UDP.
    EN: Checks telemetry delivery to both WebRTC and UDP transports.
    """
    webrtc, udp = mock_services
    data_str = '{"type": "telemetry"}'

    sender._broadcast(data_str, "telemetry")

    webrtc.send_data.assert_called_once_with(data_str, "telemetry")
    udp.send_data.assert_called_once_with(data_str)


def test_broadcast_webrtc_failure_does_not_block_udp(sender, mock_services):
    """PL: Sprawdza, że błąd WebRTC nie blokuje wysłania przez UDP.
    EN: Checks that a WebRTC failure does not block UDP delivery.
    """
    webrtc, udp = mock_services
    webrtc.send_data.side_effect = Exception("WebRTC Error")
    data_str = '{"type": "telemetry"}'

    sender._broadcast(data_str, "telemetry")

    webrtc.send_data.assert_called_once()
    udp.send_data.assert_called_once_with(data_str)
