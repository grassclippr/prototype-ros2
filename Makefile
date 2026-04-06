# Shared config lives in .env (also auto-loaded by docker-compose).
-include .env
export

PIO_PROJECT := rover/baseboard
SERIAL_DEV ?= /dev/ttyACM0
PROXY_PORT ?= 8888
BAUDRATE ?= 921600
PODMAN_NETWORK ?= prototype-ros2_micro_ros_net
PODMAN_PROXY_IMAGE ?= localhost/prototype-ros2_serial_mux_proxy:latest
PODMAN_AGENT_IMAGE ?= docker.io/microros/micro-ros-agent:jazzy

.PHONY: flash
flash:
	@PIO_PROJECT=$(PIO_PROJECT) scripts/flash_with_restore.sh

.PHONY: flash-no-ros
flash-no-ros:
	@PIO_PROJECT=$(PIO_PROJECT) PIO_BUILD_FLAGS="-DSERIAL_MUX_DISABLE_ROS=1" scripts/flash_with_restore.sh

.PHONY: flash-no-mux
flash-no-mux:
	@PIO_PROJECT=$(PIO_PROJECT) PIO_BUILD_FLAGS="-DSERIAL_MUX_ENABLE=0" scripts/flash_with_restore.sh

.PHONY: flash-mux-no-debug
flash-mux-no-debug:
	@PIO_PROJECT=$(PIO_PROJECT) PIO_BUILD_FLAGS="-DSERIAL_MUX_DEBUG_ENABLE=0" scripts/flash_with_restore.sh

.PHONY: flash-mux-no-debug-skip-ping
flash-mux-no-debug-skip-ping:
	@PIO_PROJECT=$(PIO_PROJECT) PIO_BUILD_FLAGS="-DSERIAL_MUX_DEBUG_ENABLE=0 -DSERIAL_MUX_SKIP_PING=1" scripts/flash_with_restore.sh

.PHONY: ros
ros:
	@docker compose exec -T core bash -lc "source /opt/ros/jazzy/setup.bash && ros2 $(ARGS)"

.PHONY: podman-core-build
podman-core-build:
	@podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build"

.PHONY: podman-stop-proxy
podman-stop-proxy:
	@podman rm -f serial_mux_proxy >/dev/null 2>&1 || true

.PHONY: podman-start-proxy
podman-start-proxy:
	@podman run -d --name serial_mux_proxy --network $(PODMAN_NETWORK) --device $(SERIAL_DEV):$(SERIAL_DEV) -v $(CURDIR)/serial_mux_proxy.py:/app/serial_mux_proxy.py:ro -v $(CURDIR)/serial_mux.py:/app/serial_mux.py:ro $(PODMAN_PROXY_IMAGE) $(SERIAL_DEV) $(PROXY_PORT) $(BAUDRATE) --agent-host micro_ros_agent

.PHONY: podman-restart-proxy
podman-restart-proxy: podman-stop-proxy podman-start-proxy

.PHONY: podman-restart-agent-stack
podman-restart-agent-stack:
	@podman rm -f micro_ros_agent serial_mux_proxy >/dev/null 2>&1 || true
	@podman run -d --name serial_mux_proxy --network $(PODMAN_NETWORK) --device $(SERIAL_DEV):$(SERIAL_DEV) -v $(CURDIR)/serial_mux_proxy.py:/app/serial_mux_proxy.py:ro -v $(CURDIR)/serial_mux.py:/app/serial_mux.py:ro $(PODMAN_PROXY_IMAGE) $(SERIAL_DEV) $(PROXY_PORT) $(BAUDRATE) --agent-host micro_ros_agent
	@podman run -d --name micro_ros_agent --network $(PODMAN_NETWORK) $(PODMAN_AGENT_IMAGE) tcp4 --port $(PROXY_PORT) -v6

.PHONY: test
test:
	@PYTHONPATH=. uv run pytest

.PHONY: cppcheck-baseboard
cppcheck-baseboard:
	@if ! command -v cppcheck >/dev/null 2>&1; then \
		echo "error: cppcheck is not installed" >&2; \
		exit 1; \
	fi
	@cppcheck \
		--language=c++ \
		--std=c++17 \
		--enable=warning,performance,portability \
		--inline-suppr \
		--error-exitcode=1 \
		--quiet \
		--suppress=missingIncludeSystem \
		--suppress=normalCheckLevelMaxBranches \
		-I rover/baseboard/src \
		-I rover/baseboard/include \
		-I rover/baseboard/lib \
		-D ARDUINO=1 \
		-D ESP32=1 \
		-D __XTENSA__=1 \
		rover/baseboard/src

.PHONY: esp32-verify-build-flash
esp32-verify-build-flash: podman-stop-proxy cppcheck-baseboard
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -t clean_microros
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT)
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -t upload --upload-port $(SERIAL_DEV)
	@$(MAKE) podman-start-proxy
	@$(MAKE) podman-core-rebuild-msgs

.PHONY: podman-core-rebuild-msgs
podman-core-rebuild-msgs:
	@echo "Rebuilding rover_msgs + rover_description in core container..."
	@podman exec core bash -lc 'source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build --packages-select rover_msgs rover_description' 2>&1

.PHONY: e2e
e2e: flash
	@./scripts/manual_e2e.sh

.PHONY: e2e-drive
e2e-drive: flash
	@./scripts/manual_drive_command_e2e.sh
