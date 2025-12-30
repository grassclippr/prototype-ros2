"""Serial mux framing helpers (SLIP-based)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import zlib

END = 0xC0
ESC = 0xDB
ESC_END = 0xDC
ESC_ESC = 0xDD

MAGIC = 0xA7
VERSION = 0x01

TYPE_ROS = 0x01
TYPE_DEBUG = 0x02

FLAG_CHUNKED = 0x01
FLAG_CHUNK_START = 0x02
FLAG_CHUNK_END = 0x04

MAX_PAYLOAD = 256

HEADER_LEN = 1 + 1 + 1 + 1 + 2 + 2 + 2
CRC_LEN = 4
MIN_FRAME_LEN = HEADER_LEN + CRC_LEN


@dataclass
class Frame:
    frame_type: int
    flags: int
    seq: int
    msg_id: int
    payload: bytes


import threading


class MuxWriter:
    def __init__(self, stream) -> None:
        self._stream = stream
        self._lock = threading.Lock()

    def write_bytes(self, payload: bytes) -> None:
        with self._lock:
            for b in payload:
                self._stream.write(b)


def slip_encode(data: bytes) -> bytes:
    encoded = bytearray()
    for b in data:
        if b == END:
            encoded.append(ESC)
            encoded.append(ESC_END)
        elif b == ESC:
            encoded.append(ESC)
            encoded.append(ESC_ESC)
        else:
            encoded.append(b)
    encoded.append(END)
    return bytes(encoded)


def slip_decode(data: bytes) -> Optional[bytes]:
    decoded = bytearray()
    i = 0
    while i < len(data):
        b = data[i]
        if b == ESC:
            if i + 1 >= len(data):
                return None
            nxt = data[i + 1]
            if nxt == ESC_END:
                decoded.append(END)
            elif nxt == ESC_ESC:
                decoded.append(ESC)
            else:
                return None
            i += 2
            continue
        decoded.append(b)
        i += 1
    return bytes(decoded)


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def build_frame(frame_type: int, flags: int, seq: int, msg_id: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(payload)} > {MAX_PAYLOAD}")

    header = bytearray()
    header.append(MAGIC)
    header.append(VERSION)
    header.append(frame_type & 0xFF)
    header.append(flags & 0xFF)
    header += int(seq & 0xFFFF).to_bytes(2, "little")
    header += int(msg_id & 0xFFFF).to_bytes(2, "little")
    header += int(len(payload)).to_bytes(2, "little")

    crc = crc32(bytes(header[1:]) + payload)
    frame = bytes(header) + payload + crc.to_bytes(4, "little")
    return slip_encode(frame)


def parse_frame(decoded: bytes) -> Optional[Frame]:
    return _parse_frame(decoded)[0]


def _parse_frame(decoded: bytes) -> tuple[Optional[Frame], str]:
    if len(decoded) < MIN_FRAME_LEN:
        return None, "too_short"
    if decoded[0] != MAGIC:
        return None, "bad_magic"
    if decoded[1] != VERSION:
        return None, "bad_version"

    frame_type = decoded[2]
    flags = decoded[3]
    seq = int.from_bytes(decoded[4:6], "little")
    msg_id = int.from_bytes(decoded[6:8], "little")
    length = int.from_bytes(decoded[8:10], "little")

    if length > MAX_PAYLOAD:
        return None, "length_too_large"
    expected_len = HEADER_LEN + length + CRC_LEN
    if len(decoded) != expected_len:
        return None, "length_mismatch"

    payload = decoded[10:10 + length]
    crc_read = int.from_bytes(decoded[10 + length:10 + length + 4], "little")
    crc_calc = crc32(decoded[1:10] + payload)
    if crc_calc != crc_read:
        return None, "crc_mismatch"

    return (
        Frame(
            frame_type=frame_type,
            flags=flags,
            seq=seq,
            msg_id=msg_id,
            payload=payload,
        ),
        "ok",
    )
