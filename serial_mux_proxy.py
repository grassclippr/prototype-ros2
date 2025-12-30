#!/usr/bin/env python3
"""Serial mux proxy between an ESP32 and the micro-ROS agent."""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from typing import Callable, Dict, Optional

import serial

from serial_mux import (
    END,
    FLAG_CHUNKED,
    FLAG_CHUNK_END,
    FLAG_CHUNK_START,
    Frame,
    TYPE_DEBUG,
    TYPE_ROS,
    build_frame,
    parse_frame,
    _parse_frame,
    slip_decode,
    MAX_PAYLOAD,
)


def _dump_debug(data: bytes) -> None:
    try:
        text = data.decode("utf-8")
        print(text, end="" if text.endswith("\n") else "\n", flush=True)
        return
    except UnicodeDecodeError:
        pass

    hex_str = " ".join(f"{b:02X}" for b in data)
    print(f"[DEBUG] {hex_str}")

def _hexdump(data: bytes, max_len: int = 64) -> str:
    if not data:
        return ""
    trimmed = data[:max_len]
    return " ".join(f"{b:02X}" for b in trimmed)


class SerialMuxProxy:
    def __init__(
        self,
        serial_dev: str,
        baudrate: int,
        agent_host: str,
        agent_port: int,
        *,
        time_fn: Callable[[], float] = time.monotonic,
        reassembly_timeout: float = 1.0,
        max_reassembly_bytes: int = 4096,
        max_raw_buffer_bytes: int = 65536,
        stats_interval: float = 10.0,
        dump_ros: int = 0,
        dump_bad_frames: int = 0,
    ) -> None:
        self.serial_dev = serial_dev
        self.baudrate = baudrate
        self.agent_host = agent_host
        self.agent_port = agent_port
        self.time_fn = time_fn
        self.reassembly_timeout = reassembly_timeout
        self.max_reassembly_bytes = max_reassembly_bytes
        self.max_raw_buffer_bytes = max_raw_buffer_bytes
        self.stats_interval = stats_interval
        self.dump_ros = dump_ros
        self.dump_bad_frames = dump_bad_frames

        self.serial_port: Optional[serial.Serial] = None
        self.agent_socket: Optional[socket.socket] = None

        self.running = False
        self.seq = 0
        self.ros_msg_id = 0
        self.debug_buffers: Dict[int, bytearray] = {}
        self.ros_buffers: Dict[int, bytearray] = {}
        self.debug_timestamps: Dict[int, float] = {}
        self.ros_timestamps: Dict[int, float] = {}
        self.stats = {
            "raw_frames": 0,
            "slip_decode_fail": 0,
            "frame_parse_fail": 0,
            "frame_parse_too_short": 0,
            "frame_parse_bad_magic": 0,
            "frame_parse_bad_version": 0,
            "frame_parse_length_too_large": 0,
            "frame_parse_length_mismatch": 0,
            "frame_parse_crc_mismatch": 0,
            "frames_ros": 0,
            "frames_debug": 0,
            "debug_reassembly_drop": 0,
            "ros_reassembly_drop": 0,
            "raw_buffer_drop": 0,
        }
        self.ros_len_hist: Dict[int, int] = {}

    def start(self) -> None:
        self.serial_port = serial.Serial(self.serial_dev, self.baudrate, timeout=0.1)
        print(f"Serial mux proxy started")
        print(f"Serial device: {self.serial_dev} @ {self.baudrate}")
        print(f"Agent: {self.agent_host}:{self.agent_port}")

        reconnect_delay = 1.0
        reconnect_max = 30.0

        self.running = True
        rx_thread = threading.Thread(target=self._read_from_serial, daemon=True)
        rx_thread.start()
        stats_thread = None
        if self.stats_interval > 0:
            stats_thread = threading.Thread(target=self._stats_loop, daemon=True)
            stats_thread.start()

        while self.running:
            try:
                print("Connecting to micro-ROS agent...")
                self.agent_socket = socket.create_connection((self.agent_host, self.agent_port), timeout=10)
                print("Agent connected")

                tx_thread = threading.Thread(target=self._read_from_agent, daemon=True)
                tx_thread.start()

                while self.running and tx_thread.is_alive():
                    time.sleep(0.2)

                if not self.running:
                    break

                if self.agent_socket:
                    try:
                        self.agent_socket.close()
                    except Exception:
                        pass
                    self.agent_socket = None

                print(f"Agent disconnected, reconnecting in {reconnect_delay:.1f}s")
            except KeyboardInterrupt:
                break
            except Exception as exc:
                print(f"Agent connection error: {exc}. Retrying in {reconnect_delay:.1f}s")

            time.sleep(reconnect_delay)
            reconnect_delay = min(reconnect_max, reconnect_delay * 2)

        self.stop()

    def stop(self) -> None:
        self.running = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass

    def _read_from_serial(self) -> None:
        if not self.serial_port:
            return

        buffer = bytearray()
        while self.running and self.serial_port:
            try:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not data:
                    continue
                buffer.extend(data)

                while True:
                    end_idx = buffer.find(bytes([END]))
                    if end_idx == -1:
                        break

                    raw_frame = bytes(buffer[:end_idx])
                    buffer = buffer[end_idx + 1 :]
                    if not raw_frame:
                        continue

                    self.stats["raw_frames"] += 1
                    decoded = slip_decode(raw_frame)
                    if decoded is None:
                        self.stats["slip_decode_fail"] += 1
                        continue

                    frame, reason = _parse_frame(decoded)
                    if frame is None:
                        self.stats["frame_parse_fail"] += 1
                        if reason == "too_short":
                            self.stats["frame_parse_too_short"] += 1
                        elif reason == "bad_magic":
                            self.stats["frame_parse_bad_magic"] += 1
                        elif reason == "bad_version":
                            self.stats["frame_parse_bad_version"] += 1
                        elif reason == "length_too_large":
                            self.stats["frame_parse_length_too_large"] += 1
                        elif reason == "length_mismatch":
                            self.stats["frame_parse_length_mismatch"] += 1
                        elif reason == "crc_mismatch":
                            self.stats["frame_parse_crc_mismatch"] += 1
                        if self.dump_bad_frames > 0:
                            print(
                                f"[proxy] Bad frame reason={reason} decoded_len={len(decoded)} hex={_hexdump(decoded)}",
                                flush=True,
                            )
                            self.dump_bad_frames -= 1
                        continue

                    self._handle_frame(frame)
                if len(buffer) > self.max_raw_buffer_bytes:
                    buffer = self._cap_raw_buffer(buffer)
            except Exception as exc:
                print(f"Serial read error: {exc}")
                break

    def _handle_frame(self, frame: Frame) -> None:
        if frame.frame_type == TYPE_ROS:
            self.stats["frames_ros"] += 1
        # print(f"[proxy] Received ROS frame: seq={frame.seq} msg_id={frame.msg_id} len={len(frame.payload)} flags={frame.flags}", flush=True)
            self._handle_ros_frame(frame)
            return

        if frame.frame_type != TYPE_DEBUG:
            return

        self.stats["frames_debug"] += 1
        flags = frame.flags
        if flags & FLAG_CHUNKED:
            self._prune_expired(self.debug_buffers, self.debug_timestamps)
            msg_id = frame.msg_id
            if flags & FLAG_CHUNK_START:
                self.debug_buffers[msg_id] = bytearray()
            if msg_id in self.debug_buffers:
                self.debug_timestamps[msg_id] = self.time_fn()
            if msg_id not in self.debug_buffers:
                return
            self.debug_buffers[msg_id].extend(frame.payload)
            if len(self.debug_buffers[msg_id]) > self.max_reassembly_bytes:
                self.debug_buffers.pop(msg_id, None)
                self.debug_timestamps.pop(msg_id, None)
                self.stats["debug_reassembly_drop"] += 1
                return
            if flags & FLAG_CHUNK_END:
                data = bytes(self.debug_buffers.pop(msg_id, bytearray()))
                self.debug_timestamps.pop(msg_id, None)
                _dump_debug(data)
            return

        _dump_debug(frame.payload)

    def _handle_ros_frame(self, frame: Frame) -> None:
        if not self.agent_socket:
            return

        self.ros_len_hist[len(frame.payload)] = self.ros_len_hist.get(len(frame.payload), 0) + 1
        if self.dump_ros > 0:
            print(
                f"[proxy] ROS payload len={len(frame.payload)} hex={_hexdump(frame.payload)}",
                flush=True,
            )
            self.dump_ros -= 1

        flags = frame.flags
        if flags & FLAG_CHUNKED:
            self._prune_expired(self.ros_buffers, self.ros_timestamps)
            msg_id = frame.msg_id
            if flags & FLAG_CHUNK_START:
                self.ros_buffers[msg_id] = bytearray()
            if msg_id in self.ros_buffers:
                self.ros_timestamps[msg_id] = self.time_fn()
            if msg_id not in self.ros_buffers:
                return
            self.ros_buffers[msg_id].extend(frame.payload)
            if len(self.ros_buffers[msg_id]) > self.max_reassembly_bytes:
                self.ros_buffers.pop(msg_id, None)
                self.ros_timestamps.pop(msg_id, None)
                self.stats["ros_reassembly_drop"] += 1
                return
            if flags & FLAG_CHUNK_END:
                payload = bytes(self.ros_buffers.pop(msg_id, bytearray()))
                self.ros_timestamps.pop(msg_id, None)
                self._send_to_agent(payload)
            return

        self._send_to_agent(frame.payload)

    def _send_to_agent(self, payload: bytes) -> None:
        if not self.agent_socket:
            return
        length_prefix = len(payload).to_bytes(2, "little")
        try:
            self.agent_socket.sendall(length_prefix + payload)
        except Exception as exc:
            print(f"Agent send error: {exc}")

    def _cap_raw_buffer(self, buffer: bytearray) -> bytearray:
        last_end = buffer.rfind(bytes([END]))
        if last_end == -1:
            self.stats["raw_buffer_drop"] += 1
            return bytearray()
        if last_end + 1 < len(buffer):
            self.stats["raw_buffer_drop"] += 1
        return buffer[last_end + 1 :]

    def _prune_expired(self, buffers: Dict[int, bytearray], timestamps: Dict[int, float]) -> None:
        if self.reassembly_timeout <= 0:
            return
        now = self.time_fn()
        expired = [msg_id for msg_id, ts in timestamps.items() if now - ts > self.reassembly_timeout]
        for msg_id in expired:
            buffers.pop(msg_id, None)
            timestamps.pop(msg_id, None)

    def _stats_loop(self) -> None:
        while self.running:
            time.sleep(self.stats_interval)
            if not self.running:
                break
            stats = " ".join(f"{k}={v}" for k, v in self.stats.items())
            if self.ros_len_hist:
                ros_lens = ",".join(f"{k}:{v}" for k, v in sorted(self.ros_len_hist.items()))
                stats = f"{stats} ros_len_hist={ros_lens}"
            print(f"[proxy-stats] {stats}", flush=True)

    def _read_from_agent(self) -> None:
        if not self.agent_socket:
            return

        buffer = bytearray()
        while self.running and self.agent_socket:
            try:
                data = self.agent_socket.recv(1024)
                if not data:
                    break
                # print(f"[proxy] Read {len(data)} bytes from agent", flush=True)
                buffer.extend(data)

                while len(buffer) >= 2:
                    msg_len = int.from_bytes(buffer[0:2], "little")
                    if len(buffer) < 2 + msg_len:
                        break
                    payload = bytes(buffer[2:2 + msg_len])
                    buffer = buffer[2 + msg_len :]
                    # print(f"[proxy] Forwarding {len(payload)} bytes from agent to serial", flush=True)
                    self._send_ros_to_serial(payload)
            except TimeoutError:
                continue
            except Exception as exc:
                print(f"Agent read error: {exc}")
                break

    def _send_ros_to_serial(self, payload: bytes) -> None:
        if not self.serial_port:
            return

        msg_id = 0
        if len(payload) > MAX_PAYLOAD:
            self.ros_msg_id = (self.ros_msg_id + 1) & 0xFFFF
            msg_id = self.ros_msg_id

        offset = 0
        while offset < len(payload):
            chunk = payload[offset : offset + MAX_PAYLOAD]
            offset += len(chunk)
            flags = 0
            if len(payload) > MAX_PAYLOAD:
                flags |= FLAG_CHUNKED
                if offset - len(chunk) == 0:
                    flags |= FLAG_CHUNK_START
                if offset >= len(payload):
                    flags |= FLAG_CHUNK_END

            frame = build_frame(
                frame_type=TYPE_ROS,
                flags=flags,
                seq=self.seq,
                msg_id=msg_id,
                payload=chunk,
            )
            self.seq = (self.seq + 1) & 0xFFFF
            self.serial_port.write(frame)
            # print(f"[proxy] Wrote frame to serial: seq={self.seq} len={len(frame)}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serial mux proxy for micro-ROS")
    parser.add_argument("serial_dev", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("agent_port", nargs="?", type=int, default=8888)
    parser.add_argument("baudrate", nargs="?", type=int, default=115200)
    parser.add_argument("--agent-host", default="127.0.0.1")
    parser.add_argument("--stats-interval", type=float, default=10.0)
    parser.add_argument("--dump-ros", type=int, default=0, help="Hex dump first N ROS payloads.")
    parser.add_argument("--dump-bad-frames", type=int, default=0, help="Hex dump first N bad frames.")

    args = parser.parse_args()

    proxy = SerialMuxProxy(
        serial_dev=args.serial_dev,
        baudrate=args.baudrate,
        agent_host=args.agent_host,
        agent_port=args.agent_port,
        stats_interval=args.stats_interval,
        dump_ros=args.dump_ros,
        dump_bad_frames=args.dump_bad_frames,
    )
    try:
        proxy.start()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
