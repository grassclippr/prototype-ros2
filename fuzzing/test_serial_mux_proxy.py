from __future__ import annotations

from dataclasses import dataclass
import io
import threading
import time

import pytest

from serial_mux import (
    END,
    FLAG_CHUNKED,
    FLAG_CHUNK_END,
    FLAG_CHUNK_START,
    Frame,
    TYPE_DEBUG,
    TYPE_ROS,
    build_frame,
)
from serial_mux_proxy import SerialMuxProxy


@dataclass
class DummySocket:
    sent: list[bytes]

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)


class FakeClock:
    def __init__(self) -> None:
        self._now = 0.0

    def now(self) -> float:
        return self._now

    def advance(self, seconds: float) -> None:
        self._now += seconds


def _frame(frame_type: int, flags: int, msg_id: int, payload: bytes) -> Frame:
    return Frame(frame_type=frame_type, flags=flags, seq=1, msg_id=msg_id, payload=payload)


def test_ros_chunk_timeout_discards() -> None:
    clock = FakeClock()
    proxy = SerialMuxProxy(
        serial_dev="/dev/null",
        baudrate=115200,
        agent_host="127.0.0.1",
        agent_port=8888,
        time_fn=clock.now,
        reassembly_timeout=0.5,
    )
    dummy = DummySocket(sent=[])
    proxy.agent_socket = dummy

    msg_id = 7
    proxy._handle_frame(_frame(TYPE_ROS, FLAG_CHUNKED | FLAG_CHUNK_START, msg_id, b"abc"))
    clock.advance(1.0)
    proxy._handle_frame(_frame(TYPE_ROS, FLAG_CHUNKED | FLAG_CHUNK_END, msg_id, b"def"))

    assert dummy.sent == []


def test_debug_chunk_overflow_discards() -> None:
    clock = FakeClock()
    proxy = SerialMuxProxy(
        serial_dev="/dev/null",
        baudrate=115200,
        agent_host="127.0.0.1",
        agent_port=8888,
        time_fn=clock.now,
        max_reassembly_bytes=4,
    )

    msg_id = 3
    proxy._handle_frame(_frame(TYPE_DEBUG, FLAG_CHUNKED | FLAG_CHUNK_START, msg_id, b"ab"))
    proxy._handle_frame(_frame(TYPE_DEBUG, FLAG_CHUNKED, msg_id, b"cd"))
    proxy._handle_frame(_frame(TYPE_DEBUG, FLAG_CHUNKED | FLAG_CHUNK_END, msg_id, b"ef"))

    assert msg_id not in proxy.debug_buffers


def test_plaintext_interleaved_with_ros_frames_only_forwards_ros() -> None:
    """Feed the proxy handler a mix of valid ROS frames and plaintext disguised
    as debug. Verify only ROS payloads reach the agent socket."""
    clock = FakeClock()
    proxy = SerialMuxProxy(
        serial_dev="/dev/null",
        baudrate=115200,
        agent_host="127.0.0.1",
        agent_port=8888,
        time_fn=clock.now,
    )
    dummy = DummySocket(sent=[])
    proxy.agent_socket = dummy

    ros_payload = b"\x01\x02\x03\x04"
    debug_payload = b"Guru Meditation Error: Core 0 panic"

    # Simulate receiving frames in order: debug, ros, debug
    proxy._handle_frame(_frame(TYPE_DEBUG, 0, 0, debug_payload))
    proxy._handle_frame(_frame(TYPE_ROS, 0, 0, ros_payload))
    proxy._handle_frame(_frame(TYPE_DEBUG, 0, 0, debug_payload))

    # Only the ROS payload should have been forwarded
    assert len(dummy.sent) == 1
    # Sent data is length-prefixed
    expected = len(ros_payload).to_bytes(2, "little") + ros_payload
    assert dummy.sent[0] == expected


def test_corrupted_frame_does_not_forward_to_agent() -> None:
    """Build a valid frame, corrupt a byte inside it, feed through the proxy
    parser logic, and verify nothing is forwarded to the agent."""
    clock = FakeClock()
    proxy = SerialMuxProxy(
        serial_dev="/dev/null",
        baudrate=115200,
        agent_host="127.0.0.1",
        agent_port=8888,
        time_fn=clock.now,
    )
    dummy = DummySocket(sent=[])
    proxy.agent_socket = dummy

    payload = b"hello ros"
    frame_bytes = build_frame(TYPE_ROS, 0, 1, 0, payload)

    # Corrupt a byte in the middle of the SLIP-encoded frame
    corrupted = bytearray(frame_bytes)
    mid = len(corrupted) // 2
    corrupted[mid] ^= 0xFF

    # Simulate what _read_from_serial does: split on END, decode, parse
    from serial_mux import slip_decode, _parse_frame

    # Extract inner content (between first and last END)
    parts = bytes(corrupted).split(bytes([END]))
    for part in parts:
        if not part:
            continue
        decoded = slip_decode(part)
        if decoded is None:
            continue
        frame, _reason = _parse_frame(decoded)
        if frame is not None:
            proxy._handle_frame(frame)

    # Nothing should have been forwarded
    assert dummy.sent == []


def test_valid_frame_after_corruption_still_forwarded() -> None:
    """After a corrupted frame, a subsequent valid frame must still be forwarded."""
    clock = FakeClock()
    proxy = SerialMuxProxy(
        serial_dev="/dev/null",
        baudrate=115200,
        agent_host="127.0.0.1",
        agent_port=8888,
        time_fn=clock.now,
    )
    dummy = DummySocket(sent=[])
    proxy.agent_socket = dummy

    payload = b"valid data"
    valid_frame = build_frame(TYPE_ROS, 0, 2, 0, payload)

    # Build a corrupted frame
    bad_frame = bytearray(build_frame(TYPE_ROS, 0, 1, 0, b"bad"))
    bad_frame[len(bad_frame) // 2] ^= 0xFF

    # Simulate serial stream: corrupted then valid
    stream = bytes(bad_frame) + valid_frame

    from serial_mux import slip_decode, _parse_frame

    parts = stream.split(bytes([END]))
    for part in parts:
        if not part:
            continue
        decoded = slip_decode(part)
        if decoded is None:
            continue
        frame, _reason = _parse_frame(decoded)
        if frame is not None:
            proxy._handle_frame(frame)

    # The valid frame should have been forwarded
    assert len(dummy.sent) == 1
    expected = len(payload).to_bytes(2, "little") + payload
    assert dummy.sent[0] == expected
