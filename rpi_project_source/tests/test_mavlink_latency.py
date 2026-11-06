"""PL: Testy ograniczania kolejki odbiorczej i sprawiedliwości pętli.

EN: Tests for receive-backlog limits and loop scheduler fairness.
"""

from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from core import mavlink_service


def run_one_cycle(monkeypatch, messages, *, clock=None, handler=None):
    """PL: Uruchamia jedną iterację pętli odbiorczej z kontrolowanymi mockami.

    EN: Run one receive-loop cycle with controlled mocks.
    """
    service = mavlink_service.MAVLinkService()
    service.running = True
    pending = deque(messages)
    service.master = SimpleNamespace(
        recv_match=lambda **kwargs: pending.popleft() if pending else None
    )
    service.send_heartbeat = Mock()
    service._handle_message = handler or Mock()
    monkeypatch.setattr(mavlink_service.time, "monotonic", clock or (lambda: 0.0))
    sleeps = []

    def stop_at_yield(delay):
        """PL: Zatrzymuje usługę po oddaniu sterowania przez pętlę.

        EN: Stop the service when the loop yields control.
        """
        sleeps.append(delay)
        service.running = False

    monkeypatch.setattr(mavlink_service.time, "sleep", stop_at_yield)
    service._update_loop()
    return service, pending, sleeps


@pytest.mark.parametrize("count", [1, 150, 256, 600])
def test_receive_burst_before_yield(monkeypatch, count):
    """PL: Sprawdza limit obsługi wiadomości przed oddaniem sterowania.

    EN: Verify the message-processing limit before yielding control.
    """
    message = SimpleNamespace(get_type=lambda: "RC_CHANNELS_OVERRIDE")
    service, pending, sleeps = run_one_cycle(monkeypatch, [message] * count)
    assert service._rx_msg_count == min(count, 256)
    assert service._handle_message.call_count == min(count, 256)
    assert len(pending) == max(0, count - 256)
    assert sleeps == [0.0]
    service.send_heartbeat.assert_called_once()


def test_slow_handler_limits_receive_batch(monkeypatch):
    """PL: Sprawdza skrócenie partii przy wolnym handlerze.

    EN: Verify the receive batch shrinks when the handler is slow.
    """
    elapsed = [0.0]
    message = SimpleNamespace(get_type=lambda: "RC_CHANNELS_OVERRIDE")

    def handle(_message):
        """PL: Symuluje handler zużywający czas procesora.

        EN: Simulate a message handler that consumes processing time.
        """
        elapsed[0] += 0.006

    service, pending, sleeps = run_one_cycle(
        monkeypatch, [message] * 20, clock=lambda: elapsed[0], handler=handle
    )
    assert service._rx_msg_count == 1
    assert len(pending) == 19
    assert sleeps == [0.0]


def test_empty_receive_backs_off(monkeypatch):
    """PL: Sprawdza krótkie opóźnienie po pustym odbiorze.

    EN: Verify a short backoff after an empty receive.
    """
    service, pending, sleeps = run_one_cycle(monkeypatch, [])
    assert service._rx_msg_count == 0
    assert sleeps == [0.01]
