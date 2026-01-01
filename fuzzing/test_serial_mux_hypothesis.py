from __future__ import annotations


from hypothesis import given, settings
from hypothesis import strategies as st

from serial_mux import (
    END,
    ESC,
    ESC_END,
    ESC_ESC,
    Frame,
    TYPE_DEBUG,
    TYPE_ROS,
    build_frame,
    parse_frame,
    slip_decode,
    slip_encode,
    MAX_PAYLOAD,
)


def _scan_frames(stream: bytes) -> list[Frame]:
    frames = []
    buf = bytearray()
    for b in stream:
        if b == END:
            if not buf:
                continue
            decoded = slip_decode(bytes(buf))
            buf.clear()
            if decoded is None:
                continue
            frame = parse_frame(decoded)
            if frame is None:
                continue
            frames.append(frame)
            continue
        buf.append(b)
    return frames


def _reassemble(frames: list[Frame]) -> list[bytes]:
    assembled = []
    buffers: dict[int, bytearray] = {}
    for frame in frames:
        if frame.flags & 0x01:
            msg_id = frame.msg_id
            if frame.flags & 0x02:
                buffers[msg_id] = bytearray()
            if msg_id not in buffers:
                continue
            buffers[msg_id].extend(frame.payload)
            if frame.flags & 0x04:
                assembled.append(bytes(buffers.pop(msg_id)))
        else:
            assembled.append(frame.payload)
    return assembled


@given(st.binary())
@settings(max_examples=200)
def test_slip_roundtrip(data: bytes) -> None:
    encoded = slip_encode(data)
    assert encoded.endswith(bytes([END]))
    decoded = slip_decode(encoded[:-1])
    assert decoded == data


@given(
    st.binary(min_size=0, max_size=MAX_PAYLOAD),
    st.integers(min_value=0, max_value=0xFFFF),
    st.integers(min_value=0, max_value=0xFFFF),
    st.sampled_from([TYPE_ROS, TYPE_DEBUG]),
    st.integers(min_value=0, max_value=0xFF),
)
@settings(max_examples=200)
def test_frame_roundtrip(payload: bytes, seq: int, msg_id: int, frame_type: int, flags: int) -> None:
    encoded = build_frame(frame_type, flags, seq, msg_id, payload)
    decoded = slip_decode(encoded[:-1])
    assert decoded is not None
    frame = parse_frame(decoded)
    assert frame is not None
    assert frame.payload == payload
    assert frame.seq == (seq & 0xFFFF)
    assert frame.msg_id == (msg_id & 0xFFFF)
    assert frame.frame_type == frame_type


@given(
    st.binary(min_size=0, max_size=MAX_PAYLOAD),
    st.integers(min_value=0, max_value=0xFFFF),
)
@settings(max_examples=200)
def test_crc_rejects_tampered(payload: bytes, seq: int) -> None:
    encoded = build_frame(TYPE_ROS, 0, seq, 0, payload)
    raw = bytearray(encoded[:-1])
    if raw:
        raw[len(raw) // 2] ^= 0x01
    decoded = slip_decode(bytes(raw))
    if decoded is None:
        return
    frame = parse_frame(decoded)
    assert frame is None


@given(
    payload=st.binary(min_size=1, max_size=MAX_PAYLOAD),
    noise=st.binary(min_size=1, max_size=16),
)
@settings(max_examples=200)
def test_resync_with_noise(payload: bytes, noise: bytes) -> None:
    frame = build_frame(TYPE_ROS, 0, 1, 0, payload)
    stream = noise + bytes([END]) + frame + noise + bytes([END]) + frame
    frames = _scan_frames(stream)
    assert len(frames) >= 1
    assert frames[0].payload == payload


@given(st.binary(min_size=MAX_PAYLOAD + 1, max_size=MAX_PAYLOAD * 3))
@settings(max_examples=100)
def test_chunk_reassembly(payload: bytes) -> None:
    msg_id = 42
    frames = []
    offset = 0
    while offset < len(payload):
        chunk = payload[offset : offset + MAX_PAYLOAD]
        flags = 0x01
        if offset == 0:
            flags |= 0x02
        if offset + len(chunk) >= len(payload):
            flags |= 0x04
        encoded = build_frame(TYPE_DEBUG, flags, 1, msg_id, chunk)
        decoded = slip_decode(encoded[:-1])
        assert decoded is not None
        frame = parse_frame(decoded)
        assert frame is not None
        frames.append(frame)
        offset += len(chunk)

    assembled = _reassemble(frames)
    assert assembled == [payload]
