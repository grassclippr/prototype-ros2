#!/usr/bin/env python3
"""
Micro-ROS Proxy for ESP32 Communication

Usage:
  python3 microros_proxy.py [SERIAL_DEV] [PROXY_PORT] [BAUDRATE]

  All arguments are optional and can also be set via environment variables:
    SERIAL_DEV: serial device path (default: /dev/ttyACM0)
    PROXY_PORT: TCP port for agent connection (default: 8888)
    BAUDRATE:   Serial baudrate (default: 115200)

  Example:
    SERIAL_DEV=/dev/ttyUSB0 PROXY_PORT=9000 BAUDRATE=230400 python3 microros_proxy.py
    python3 microros_proxy.py /dev/ttyUSB0 9000 230400

This proxy sits between an ESP32 running micro-ROS and the micro-ROS agent.
It routes messages based on magic byte prefixes:
- ROS messages are forwarded to the micro-ROS agent
- Debug/crash dump messages are printed to console
"""

import os
import serial
import socket
import threading
import time
import sys
from typing import Optional

ROS_MAGIC_BYTE = 0x00

BEGIN_FLAG = 0x7E
ESCAPE_FLAG = 0x7D
XOR_FLAG = 0x20


_CRC16_FCSTAB = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
]

def debug_print_bytes(title: str, data: bytes):
    # Print title in green
    print(f"\033[32m{title}:\033[0m", end=' ')
    # Print bytes, highlight 0x00 in red
    byte_strs = []
    for b in data:
        if b == 0x7E:
            byte_strs.append(f"\033[31m{b:02X}\033[0m")
        else:
            byte_strs.append(f"{b:02X}")
    print(' '.join(byte_strs))

class DecodeError(Exception):
    pass

class CobsCodec:
    @staticmethod
    def decode(in_bytes):
        if isinstance(in_bytes, str):
            raise TypeError('Unicode-objects are not supported; byte buffer objects only')
        mv = memoryview(in_bytes)
        if mv.ndim > 1 or mv.itemsize > 1:
            raise BufferError('object must be a single-dimension buffer of bytes.')
        try:
            if mv.format != 'B':
                mv = mv.cast('B')
        except AttributeError:
            pass
        out_bytes = bytearray()
        idx = 0
        if len(mv) > 0:
            while True:
                length = mv[idx]
                if length == 0:
                    raise DecodeError("zero byte found in input")
                idx += 1
                end = idx + length - 1
                copy_mv = mv[idx:end]
                if 0 in copy_mv:
                    raise DecodeError("zero byte found in input")
                out_bytes += copy_mv
                idx = end
                if idx > len(mv):
                    raise DecodeError("not enough input bytes for length code")
                if idx < len(mv):
                    if length < 0xFF:
                        out_bytes.append(0)
                else:
                    break
        return bytes(out_bytes)

class FrameStuffing:
    @staticmethod
    def stuff(frame: bytes) -> bytearray:
        stuffed = bytearray()
        for b in frame:
            if b in (BEGIN_FLAG, ESCAPE_FLAG):
                stuffed.append(ESCAPE_FLAG)
                stuffed.append(b ^ XOR_FLAG)
            else:
                stuffed.append(b)
        return stuffed

    @staticmethod
    def unstuff(stuffed: bytes) -> bytearray:
        unstuffed = bytearray()
        i = 0
        while i < len(stuffed):
            b = stuffed[i]
            if b == ESCAPE_FLAG:
                if i + 1 >= len(stuffed):
                    raise ValueError("Incomplete escape sequence in frame")
                next_b = stuffed[i + 1]
                unstuffed.append(next_b ^ XOR_FLAG)
                i += 2
            else:
                unstuffed.append(b)
                i += 1
        return unstuffed

class HDLCFrame:
    @staticmethod
    def crc16_ccitt(data: bytes) -> int:
        fcstab = _CRC16_FCSTAB
        crc = 0x0000
        for b in data:
            crc = ((crc >> 8) & 0xFFFF) ^ fcstab[(crc ^ b) & 0xFF]
        return crc & 0xFFFF

    @staticmethod
    def build(source_addr: int, remote_addr: int, payload: bytes) -> bytearray:
        frame = bytearray()
        frame.append(source_addr)
        frame.append(remote_addr)
        frame += payload
        crc = HDLCFrame.crc16_ccitt(payload[2:]) # dont crc the length bytes
        frame += crc.to_bytes(2, 'little')
        return frame

    @staticmethod
    def parse(unstuffed: bytes):
        # debug_print_bytes("Parsing unstuffed:", unstuffed)

        if len(unstuffed) < 6:
            return {
                'error': f"expected at least 6 bytes, got {len(unstuffed)}"
            }
        
        # Find the start of the frame
        start = unstuffed.find(BEGIN_FLAG)
        if start == -1:
            return {
                'error': "No BEGIN_FLAG found in frame"
            }
        unstuffed = unstuffed[start + 1:]

        payload_len = int.from_bytes(unstuffed[2:4], 'little')
        expected_total = 4 + payload_len + 2
        if len(unstuffed) < expected_total:
            return {
                'error': f"expected {expected_total} bytes, got {len(unstuffed)}"
            }
        payload = bytes(unstuffed[4:4 + payload_len])

        # Check that the payload does not contain BEGIN_FLAG
        if BEGIN_FLAG in payload:
            # Return the last_byte_index to allow caller to skip this invalid frame
            return {
                'last_byte_index': 4 + payload_len + 2,
                'error': f"Invalid payload: contains BEGIN_FLAG"
            }

        # CRC check payload integrity
        frame_crc = int.from_bytes(unstuffed[4 + payload_len:4 + payload_len + 2], 'little')
        calc_crc = HDLCFrame.crc16_ccitt(payload)
        if calc_crc != frame_crc:
            return {
                'last_byte_index': 4 + payload_len + 2,
                'error': f"CRC mismatch: calc=0x{calc_crc:04X}, frame=0x{frame_crc:04X}"
            }

        return {
            'source_addr': unstuffed[0],
            'remote_addr': unstuffed[1],
            'payload_len': payload_len,
            'payload': unstuffed[2:4] + payload,
            'crc': frame_crc,
            'last_byte_index': 4 + payload_len + 2
        }


class MicroROSProxy:
    last_remote = 0x00
    last_source = 0x00

    def __init__(self, esp32_port: str = "/dev/ttyACM0",
                 esp32_baudrate: int = 115200,
                 agent_host: str = "localhost",
                 agent_port: int = 8888):
        self.esp32_port = esp32_port
        self.esp32_baudrate = esp32_baudrate
        self.agent_host = agent_host
        self.agent_port = agent_port

        self.esp32_serial: Optional[serial.Serial] = None
        self.agent_socket: Optional[socket.socket] = None
        self.server_socket: Optional[socket.socket] = None

        self.running = False

    def start(self):
        """Start the proxy service"""
        print(f"Starting Micro-ROS Proxy...")
        print(f"ESP32: {self.esp32_port} @ {self.esp32_baudrate} baud")
        print(f"Agent: {self.agent_host}:{self.agent_port}")
        print(f"Magic bytes - ROS: 0x{ROS_MAGIC_BYTE:02X}")

        try:
            # Connect to ESP32
            self.esp32_serial = serial.Serial(
                self.esp32_port,
                #self.esp32_baudrate,
                timeout=0.1
            )

            print(f"✓ Connected to ESP32 on {self.esp32_port}")

            # Agent reconnect configuration (env overrides)
            reconnect_delay = float(os.environ.get("AGENT_RECONNECT_DELAY", "1.0"))
            reconnect_max = float(os.environ.get("AGENT_RECONNECT_MAX", "30.0"))

            self.running = True

            # Start ESP32 reading thread
            esp32_thread = threading.Thread(target=self._read_from_esp32, daemon=True)
            esp32_thread.start()

            print(f"Connecting to micro-ROS agent at {self.agent_host}:{self.agent_port}...")

            # Outbound connection loop with exponential backoff
            while self.running:
                try:
                    self.agent_socket = socket.create_connection((self.agent_host, self.agent_port), timeout=10)
                    print(f"✓ Connected to micro-ROS agent at {self.agent_host}:{self.agent_port}")

                    # Reset reconnect delay after successful connect
                    reconnect_delay = float(os.environ.get("AGENT_RECONNECT_DELAY", "1.0"))

                    # Start agent reading thread
                    agent_thread = threading.Thread(target=self._read_from_agent, daemon=True)
                    agent_thread.start()

                    print("🚀 Proxy is running! Press Ctrl+C to stop.")

                    # Wait while agent thread is alive or until shutdown
                    while self.running and agent_thread.is_alive():
                        time.sleep(0.1)

                    # If shutting down, break out
                    if not self.running:
                        break

                    # Agent disconnected — cleanup and prepare to reconnect
                    if self.agent_socket:
                        try:
                            self.agent_socket.close()
                        except Exception:
                            pass
                        self.agent_socket = None

                    print(f"⚠️  Agent disconnected, will attempt to reconnect in {reconnect_delay:.1f}s")

                except KeyboardInterrupt:
                    print("\n⏹️  Stopping proxy...")
                    break
                except Exception as e:
                    print(f"❌ Agent connection error: {e}. Retrying in {reconnect_delay:.1f}s")

                # Wait before retrying (exponential backoff)
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_max, reconnect_delay * 2)

        except KeyboardInterrupt:
            print("\n⏹️  Stopping proxy...")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop the proxy service"""
        self.running = False

        if self.esp32_serial:
            self.esp32_serial.close()
            print("✓ ESP32 connection closed")

        if self.agent_socket:
            self.agent_socket.close()
            print("✓ Agent connection closed")

        if self.server_socket:
            self.server_socket.close()
            print("✓ Server socket closed")

    def _read_from_esp32(self):
        """Read data from ESP32 and route based on magic byte"""
        buffer = bytearray()
        binary_buffer = bytearray()
        last_error = None

        while self.running and self.esp32_serial:
            try:
                # Read available data
                data = self.esp32_serial.read(self.esp32_serial.in_waiting or 1)
                if not data:
                    continue

                buffer += data

                while True:
                    # Look for start of binary frame
                    start = buffer.find(b'\x00\x00')
                    if start == -1:
                        if len(buffer) == 1 and buffer[0] == 0x00:
                            # Single 0x00 byte, wait for more data
                            break

                        # No binary frame, print all as text and clear buffer
                        if buffer:
                            print(buffer.decode('utf-8', errors='replace'), end='', flush=True)
                            buffer.clear()
                        break

                    # Print any text before binary frame
                    if start > 0:
                        print(buffer[:start].decode('utf-8', errors='replace'), end='', flush=True)

                    # Look for end of binary frame
                    end = buffer.find(b'\x00', start + 2)
                    if end == -1:
                        # Wait for more data
                        break

                    # Extract binary frame
                    binary_frame = buffer[start + 2:end]

                    # Process binary frame (COBS decode, etc.)
                    try:
                        decoded = CobsCodec.decode(binary_frame)
                        binary_buffer += decoded
                    except DecodeError as e:
                        print(f"❌ COBS decode error: {e}")
                    
                    buffer = buffer[end + 1:]

                    # split on 0xFE and loop thru each frame. If parse fails, go to next
                    frame_start = 0
                    while True:
                        frame_start = binary_buffer.find(b'\x7E', frame_start)
                        if frame_start == -1:
                            break

                        frame_end = binary_buffer.find(b'\x7E', frame_start + 1)
                        if frame_end == -1:
                            frame_end = len(binary_buffer)

                        if frame_start > 0:
                            debug_print_bytes("Discarding bytes before frame", binary_buffer[:frame_start])
                            print(f"❌ Frame parse error: {last_error}")

                        frame = HDLCFrame.parse(binary_buffer[frame_start:frame_end])
                        if not frame:
                            print("❌ Failed to parse HDLC frame")
                            break

                        # If we got a valid frame, forward to ROS agent
                        if frame and 'payload' in frame:
                            #debug_print_bytes("RX frame", binary_buffer[frame_start:frame_end])
                            self.last_source = frame['source_addr']
                            self.last_remote = frame['remote_addr']
                            self._forward_to_agent(frame['payload'])

                        if 'error' in frame:
                            last_error = frame['error']
                            #print(f"❌ Frame parse error: {frame['error']}")

                        # Remove processed frame from buffer
                        if 'last_byte_index' in frame:
                            binary_buffer = binary_buffer[frame_start + frame['last_byte_index'] + 1:]
                            frame_start = 0
                        
                        # No more complete frames in buffer
                        if frame_end == len(binary_buffer):
                            break

                        frame_start = frame_end
            except Exception as e:
                print(f"❌ Error reading from ESP32: {e}")
                break
        print("⏹️  Stopped reading from ESP32")

    def _read_from_agent(self):
        """Read data from micro-ROS agent and forward to ESP32"""
        buffer = bytearray()
        while self.running and self.agent_socket:
            try:
                data = self.agent_socket.recv(1024)
                if not data:
                    break
                
                buffer += data


                # Message is formated as 2 byte little-endian length prefix + payload
                while len(buffer) >= 2:
                    msg_len = int.from_bytes(buffer[0:2], 'little')
                    if len(buffer) < 2 + msg_len:
                        # Wait for more data
                        break
                    msg = buffer[:2 + msg_len]
                    buffer = buffer[2 + msg_len:]
                
                    # Forward agent data to ESP32
                    if self.esp32_serial:
                        # Build frame as bytes for debug and atomic write
                        frame = HDLCFrame.build(self.last_source, self.last_remote, msg)

                        # Apply byte stuffing
                        stuffed = FrameStuffing.stuff(frame)

                        stuffed.insert(0, BEGIN_FLAG)

                        #debug_print_bytes("TX frame", stuffed)
                        self.esp32_serial.write(stuffed)
                    else:
                        print("❌ No ESP32 connected to forward agent data")
            except TimeoutError:
                continue
            except Exception as e:
                print(f"❌ Error reading from agent: {e}")
                break

    def _forward_to_agent(self, message: bytes):
        if not message:
            return

        try:
            if self.agent_socket:
                try:
                    self.agent_socket.send(message)
                except Exception as e:
                    print(f"❌ Error forwarding to agent: {e}")
        except Exception as e:
            print(f"❌ Error in _forward_to_agent: {e}")

def main():
    # Priority: CLI args > environment > defaults
    esp32_port = os.environ.get("SERIAL_DEV", "/dev/ttyACM0")
    proxy_port = int(os.environ.get("PROXY_PORT", "8888"))
    baudrate = int(os.environ.get("BAUDRATE", "115200"))

    # CLI overrides
    if len(sys.argv) > 1:
        esp32_port = sys.argv[1]
    if len(sys.argv) > 2:
        proxy_port = int(sys.argv[2])
    if len(sys.argv) > 3:
        baudrate = int(sys.argv[3])

    proxy = MicroROSProxy(esp32_port=esp32_port, esp32_baudrate=baudrate, agent_port=proxy_port)
    proxy.start()

if __name__ == "__main__":
    main()

