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

## Compose

```
SERIAL_DEV=/dev/ttyACM0 PROXY_PORT=8888 BAUDRATE=115200 podman-compose up
```

## macOS host proxy

On macOS, the helper scripts will prefer `podman-compose` when available, start the serial mux proxy on the host, and auto-detect common `/dev/cu.usb*` serial devices if `SERIAL_DEV` is unset or stale.

For a one-shot velocity command after the stack is up:

```
make cmd-vel LINEAR_X=0.2 ANGULAR_Z=0.0
```
