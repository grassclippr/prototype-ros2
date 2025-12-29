# Serial Mux Protocol Spec (v1, SLIP)

## Scope
- Applies to UART/serial only.
- TCP remains raw micro-ROS agent framing: `uint16_le length + payload` (no mux).

## Goals
- Robustly carry ROS and debug data on the same serial stream.
- Recover cleanly from noise/partial frames.
- Easy to test with property-based tests.

## Wire Framing (SLIP)
- END = `0xC0`, ESC = `0xDB`, ESC_END = `0xDC`, ESC_ESC = `0xDD`.
- A frame is SLIP-encoded and terminated by END.
- Encoding rules:
  - Replace END with ESC + ESC_END.
  - Replace ESC with ESC + ESC_ESC.
  - All other bytes unchanged.
- Receiver scans for END to delimit frames.

## Frame Layout (pre-SLIP)
- Magic (1 byte): `0xA7`
- Version (1 byte): `0x01`
- Type (1 byte):
  - `0x01` = ROS (opaque XRCE-DDS payload)
  - `0x02` = DEBUG (text or binary)
- Flags (1 byte):
  - bit0 = CHUNKED
  - bit1 = CHUNK_START
  - bit2 = CHUNK_END
- Seq (2 bytes, little-endian): monotonically increasing per sender
- MsgId (2 bytes, little-endian): identifies a multi-frame DEBUG message
- Length (2 bytes, little-endian): payload length (0..255 in v1)
- Payload (Length bytes)
- CRC32 (4 bytes, little-endian) over Version..Payload (Magic excluded)

## Limits
- MAX_PAYLOAD = 256 bytes in v1 (conservative).
- Any frame with Length > MAX_PAYLOAD MUST be dropped.

## Chunking (ROS and DEBUG)
- If a message is larger than MAX_PAYLOAD, split into chunks:
  - All chunks share the same MsgId.
  - Set CHUNKED on all chunks.
  - First chunk: CHUNK_START; last chunk: CHUNK_END.
- Receiver reassembles by MsgId in order of arrival.
  - If a chunk is missing or out-of-order, discard the message and log an error.

## Routing Rules
- Type=ROS: forward payload unchanged to micro-ROS agent over TCP.
- Type=DEBUG: store raw bytes or hexdump; optional UTF-8 display if desired.

## Receiver Behavior
- Read until END to get a candidate frame.
- SLIP-decode; if decode fails, drop and resync.
- Validate Magic, Version, Length, CRC32.
- Drop invalid frames without affecting sync (END delimiter restores framing).

## Sender Behavior
- MUST frame all bytes; no raw output on UART.
- MUST compute CRC and Length correctly.
- SHOULD increment Seq for each frame.

## Testing Properties (Hypothesis)
- Encode->Decode round-trip yields original frame.
- Random byte streams never crash parser and always resync on next END.
- Invalid CRC never yields accepted frame.
- Chunk reassembly accepts only complete sequences with correct ordering.
