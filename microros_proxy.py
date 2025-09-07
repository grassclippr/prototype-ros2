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
DEBUG_MAGIC_BYTE = 0xA5

class cobs_DecodeError(Exception):
    pass


def cobs_get_buffer_view(in_bytes):
    mv = memoryview(in_bytes)
    if mv.ndim > 1 or mv.itemsize > 1:
        raise BufferError('object must be a single-dimension buffer of bytes.')
    try:
        if mv.format != 'B':
            mv = mv.cast('B')
    except AttributeError:
        pass
    return mv

def cobs_decode(in_bytes):
    """Decode a string using Consistent Overhead Byte Stuffing (COBS).
    
    Input should be a byte string that has been COBS encoded. Output
    is also a byte string.
    
    A cobs.DecodeError exception will be raised if the encoded data
    is invalid."""
    if isinstance(in_bytes, str):
        raise TypeError('Unicode-objects are not supported; byte buffer objects only')
    in_bytes_mv = cobs_get_buffer_view(in_bytes)
    out_bytes = bytearray()
    idx = 0

    if len(in_bytes_mv) > 0:
        while True:
            length = in_bytes_mv[idx]
            if length == 0:
                raise cobs_DecodeError("zero byte found in input")
            idx += 1
            end = idx + length - 1
            copy_mv = in_bytes_mv[idx:end]
            if 0 in copy_mv:
                raise cobs_DecodeError("zero byte found in input")
            out_bytes += copy_mv
            idx = end
            if idx > len(in_bytes_mv):
                raise cobs_DecodeError("not enough input bytes for length code")
            if idx < len(in_bytes_mv):
                if length < 0xFF:
                    out_bytes.append(0)
            else:
                break
    return bytes(out_bytes)

class MicroROSProxy:
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
        print(f"Magic bytes - ROS: 0x{ROS_MAGIC_BYTE:02X}, Debug: 0x{DEBUG_MAGIC_BYTE:02X}")

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
        uros_msg = bytearray()
        uros_mode = False

        while self.running and self.esp32_serial:
            try:
                # Read available data
                data = self.esp32_serial.read(self.esp32_serial.in_waiting or 1)
                if not data:
                    continue

                #print("B:", ' '.join(f'{b:02X}' for b in data))
                data = buffer + data
                buffer.clear()

                # Process incoming data byte by byte
                for index, byte in enumerate(data):
                    if byte == ROS_MAGIC_BYTE:
                        if len(data[index+1:]) == 0: # If we are on the last character, add it to the buffer and wait for more data
                            buffer.append(byte)
                            uros_mode = False
                            break

                        if index > 0:
                            uros_mode = (data[index-1] == ROS_MAGIC_BYTE or data[index + 1] == ROS_MAGIC_BYTE) and byte == ROS_MAGIC_BYTE
                        else:
                            uros_mode = data[index + 1] == ROS_MAGIC_BYTE and byte == ROS_MAGIC_BYTE
                        
                    if uros_mode:
                        uros_msg.append(byte)
                    else:
                        print(chr(byte), end='', flush=True)

                # If we have uros data, decode and forward it to the agent
                if len(uros_msg) > 0:
                    try:
                        for msg in uros_msg.split(b'\x00'):
                            if len(msg) > 0:
                                decoded = cobs_decode(msg)
                                self._forward_to_agent(decoded)


                    except cobs_DecodeError as e:
                        print(f"❌ COBS decode error: {e}")

                    uros_msg.clear()
                    

                # Process complete messages in buffer
                # while len(buffer) > 0:
                    # magic_byte = buffer[0]

                    # if magic_byte == ROS_MAGIC_BYTE:
                        # # This is a ROS message - forward to agent
                        # message = self._extract_ros_message(buffer)
                        # if message:
                            # self._forward_to_agent(message)
                            # buffer = buffer[len(message):]
                        # else:
                            # break  # Need more data

                    # elif magic_byte == DEBUG_MAGIC_BYTE:
                        # # This is debug data - print to console
                        # message = self._extract_debug_message(buffer)
                        # if message:
                            # self._print_debug_message(message)
                            # buffer = buffer[len(message):]
                        # else:
                            # break  # Need more data
                    # else:
                        # # Unknown magic byte - might be micro-ROS protocol data
                        # # For now, assume it's ROS data and forward it
                        # message = self._extract_ros_message(buffer)
                        # if message:
                            # self._forward_to_agent(message)
                            # buffer = buffer[len(message):]
                        # else:
                            # # Skip this byte and continue
                            # buffer = buffer[1:]

            except Exception as e:
                print(f"❌ Error reading from ESP32: {e}")
                break
        print("⏹️  Stopped reading from ESP32")

    def _read_from_agent(self):
        """Read data from micro-ROS agent and forward to ESP32"""
        while self.running and self.agent_socket:
            try:
                data = self.agent_socket.recv(1024)
                if not data:
                    break

                # Forward agent data to ESP32
                if self.esp32_serial:
                    self.esp32_serial.write(data)

            except Exception as e:
                print(f"❌ Error reading from agent: {e}")
                break

    def _extract_ros_message(self, buffer: bytearray) -> Optional[bytes]:
        """Extract a complete ROS message from buffer"""
        # For micro-ROS, we need to understand the protocol framing
        # This is a simplified version - you may need to adjust based on actual protocol
        if len(buffer) < 4:
            return None

        # Micro-ROS typically uses length-prefixed messages
        # This is a placeholder - actual implementation depends on micro-ROS framing
        if len(buffer) >= 8:  # Minimum reasonable message size
            # For now, assume messages are reasonably small and take what we have
            return bytes(buffer[:min(len(buffer), 256)])

        return None

    def _extract_debug_message(self, buffer: bytearray) -> Optional[bytes]:
        """Extract a complete debug message from buffer"""
        # Look for newline-terminated debug messages
        try:
            newline_idx = buffer.index(b'\n'[0])
            return bytes(buffer[:newline_idx + 1])
        except ValueError:
            # No newline found - check if buffer is getting too long
            if len(buffer) > 1024:
                # Take what we have to avoid buffer overflow
                return bytes(buffer)
            return None

    def _forward_to_agent(self, message: bytes):
        """Forward ROS message to micro-ROS agent"""
        if self.agent_socket:
            try:
                self.agent_socket.send(message)
            except Exception as e:
                print(f"❌ Error forwarding to agent: {e}")
        else:
            print("❌ No agent connected")

    def _print_debug_message(self, message: bytes):
        """Print debug message to console"""
        try:
            # Remove magic byte and decode
            debug_data = message[1:].decode('utf-8', errors='ignore').strip()
            timestamp = time.strftime("%H:%M:%S")
            print(f"🐛 [{timestamp}] ESP32 DEBUG: {debug_data}")
        except Exception as e:
            # If decoding fails, print as hex
            hex_data = ' '.join(f'{b:02X}' for b in message)
            timestamp = time.strftime("%H:%M:%S")
            print(f"🐛 [{timestamp}] ESP32 DEBUG (hex): {hex_data}")

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

