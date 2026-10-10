import functools
import struct

import pytest

from core.binary_path_assembler import BinaryPathAssembler


def make_chunk(index: int, total: int, payload: bytes = b"", msg_id: int = 7) -> bytes:
    packet = b"PT" + struct.pack("<HHHH", msg_id, index, total, len(payload)) + payload
    return packet + bytes([functools.reduce(int.__xor__, packet, 0)])


@pytest.mark.parametrize("index,total", [(0, 0), (1, 1), (2, 2), (65535, 2)])
def test_invalid_header_does_not_allocate_buffer(index: int, total: int) -> None:
    assembler = BinaryPathAssembler()
    assert assembler.add_chunk(make_chunk(index, total)) is None
    assert assembler._buffers == {}


def test_conflicting_total_preserves_pending_path() -> None:
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
    assembler = BinaryPathAssembler()
    waypoint = struct.pack("<fffB", 52.0, 21.0, 10.0, 3)
    last = make_chunk(1, 2, waypoint[6:])
    assert assembler.add_chunk(last) is None
    assert assembler.add_chunk(last) is None
    assert assembler.add_chunk(make_chunk(0, 2, waypoint[:6])) == [
        {"lat": 52.0, "lon": 21.0, "alt": 10.0, "type": 3}
    ]
    assert assembler._buffers == {}
