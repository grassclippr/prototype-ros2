from __future__ import annotations

from dataclasses import dataclass

import pytest

from serial_mux import (
    FLAG_CHUNKED,
    FLAG_CHUNK_END,
    FLAG_CHUNK_START,
    Frame,
    TYPE_DEBUG,
    TYPE_ROS,
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
