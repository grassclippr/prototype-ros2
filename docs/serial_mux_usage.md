# Serial Mux Usage

## UART (proxy mode)

1) Flash the baseboard firmware with the serial mux transport enabled.
2) Run the serial mux proxy and the micro-ROS agent in TCP mode.

Example (host):

```
python3 serial_mux_proxy.py /dev/ttyACM0 8888 115200 --agent-host 127.0.0.1
```

Example (agent):

```
micro-ros-agent tcp4 --port 8888 -v6
```

## Heartbeat validation

The rover role emits a `heartbeat <n>` line once per second over the mux when `SERIAL_MUX_HEARTBEAT=1` (default).
You should see these lines in the proxy console after the rover boots.

## Docker Compose

```
SERIAL_DEV=/dev/ttyACM0 PROXY_PORT=8888 BAUDRATE=115200 docker compose up
```
