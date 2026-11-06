"""PL: Testuje walidację fragmentów i odtwarzanie podzielonej ścieżki.
EN: Tests chunk validation and reassembly of split paths.
"""

import functools
import struct

import pytest

from core.binary_path_assembler import BinaryPathAssembler


def make_chunk(index: int, total: int, payload: bytes = b"", msg_id: int = 7) -> bytes:
    """PL: Buduje pakiet fragmentu ścieżki z nagłówkiem i sumą kontrolną.
    EN: Builds a path chunk packet with a header and checksum.
    """
    packet = b"PT" + struct.pack("<HHHH", msg_id, index, total, len(payload)) + payload
    return packet + bytes([functools.reduce(int.__xor__, packet, 0)])


@pytest.mark.parametrize("index,total", [(0, 0), (1, 1), (2, 2), (65535, 2)])
def test_invalid_header_does_not_allocate_buffer(index: int, total: int) -> None:
    """PL: Sprawdza, że niepoprawny nagłówek nie tworzy bufora ścieżki.
    EN: Checks that an invalid header does not allocate a path buffer.
    """
    assembler = BinaryPathAssembler()
    assert assembler.add_chunk(make_chunk(index, total)) is None
    assert assembler._buffers == {}


def test_conflicting_total_preserves_pending_path() -> None:
    """PL: Weryfikuje, że błędne fragmenty nie uszkadzają oczekującej ścieżki.
    EN: Verifies invalid chunks do not corrupt a pending path.
    """
    assembler = BinaryPathAssembler()
    waypoint = struct.pack("<fffB", 52.0, 21.0, 10.0, 3)
    assert assembler.add_chunk(make_chunk(0, 2, waypoint[:6])) is None
    pending = assembler._buffers[7].copy()
    pending["chunks"] = pending["chunks"].copy()

    assert assembler.add_chunk(make_chunk(1, 1, b"invalid")) is None
    assert assembler._buffers[7] == pending
    assert assembler.add_chunk(make_chunk(0, 1, b"invalid")) is None
    assert assembler._buffers[7] == pending
    assert assembler.add_chunk(make_chunk(1, 3, b"invalid")) is None
    assert assembler._buffers[7] == pending

    assert assembler.add_chunk(make_chunk(1, 2, waypoint[6:])) == [
        {"lat": 52.0, "lon": 21.0, "alt": 10.0, "type": 3}
    ]
    assert assembler._buffers == {}


def test_reverse_order_and_duplicate_chunks_reassemble_split_waypoint() -> None:
    """PL: Sprawdza składanie punktu z duplikatów fragmentów w odwrotnej kolejności.
    EN: Checks waypoint reassembly from duplicate chunks received out of order.
    """
    assembler = BinaryPathAssembler()
    waypoint = struct.pack("<fffB", 52.0, 21.0, 10.0, 3)
    last = make_chunk(1, 2, waypoint[6:])
    assert assembler.add_chunk(last) is None
    assert assembler.add_chunk(last) is None
    assert assembler.add_chunk(make_chunk(0, 2, waypoint[:6])) == [
        {"lat": 52.0, "lon": 21.0, "alt": 10.0, "type": 3}
    ]
    assert assembler._buffers == {}
