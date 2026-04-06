# Shared config lives in .env (also auto-loaded by docker-compose).
-include .env
export

PIO_PROJECT := rover/baseboard
PIO_ENV := LynxAdapter_v1_0
MICROROS_HEADER_MARKER := $(PIO_PROJECT)/.pio/libdeps/$(PIO_ENV)/micro_ros_platformio/libmicroros/include/nmea_msgs/msg/sentence.h

SERIAL_DEV ?= /dev/ttyACM0
UPLOAD_PORT ?= $(SERIAL_DEV)
PROXY_PORT ?= 8888
BAUDRATE ?= 921600
JOY_DEV ?= /dev/input/js0
JOY_BACKEND ?= game_controller_node
JOY_DEVICE_ID ?= 0
JOY_DEVICE_NAME ?=
COMPOSE ?= podman-compose
PODMAN_NETWORK ?= prototype-ros2_micro_ros_net
PODMAN_PROXY_IMAGE ?= localhost/prototype-ros2_serial_mux_proxy:latest
PODMAN_AGENT_IMAGE ?= docker.io/microros/micro-ros-agent:jazzy
PODMAN_CORE_RUNTIME_IMAGE ?= localhost/prototype-ros2_core_runtime:latest
PODMAN_JOY_CONTAINER ?= joy_sidecar
CORE_PACKAGE_MANIFESTS := $(shell find rover -type f -name package.xml -not -path '*/.pio/*' | sort)
CORE_IMAGE_INPUTS := docker-compose.yml docker/Dockerfile docker/workspace.sh docker/entrypoint.sh $(CORE_PACKAGE_MANIFESTS)
CORE_CONTAINER_STAMP := .make/core-container.stamp

.PHONY: flash
flash: $(MICROROS_HEADER_MARKER)
	@PIO_PROJECT=$(PIO_PROJECT) SERIAL_DEV=$(SERIAL_DEV) UPLOAD_PORT=$(UPLOAD_PORT) PROXY_PORT=$(PROXY_PORT) scripts/flash_with_restore.sh

.PHONY: baseboard-build
baseboard-build: $(MICROROS_HEADER_MARKER)
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -e $(PIO_ENV)

$(MICROROS_HEADER_MARKER): $(PIO_PROJECT)/extra_packages/nmea_msgs/msg/Sentence.msg
	@echo "micro-ROS headers missing or outdated. Triggering build..."
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -e $(PIO_ENV)

.PHONY: baseboard-clean
baseboard-clean:
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -t clean
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -t clean_microros

.PHONY: flash-no-ros
flash-no-ros:
	@PIO_PROJECT=$(PIO_PROJECT) SERIAL_DEV=$(SERIAL_DEV) UPLOAD_PORT=$(UPLOAD_PORT) PROXY_PORT=$(PROXY_PORT) PIO_BUILD_FLAGS="-DSERIAL_MUX_DISABLE_ROS=1" scripts/flash_with_restore.sh

.PHONY: flash-no-mux
flash-no-mux:
	@PIO_PROJECT=$(PIO_PROJECT) SERIAL_DEV=$(SERIAL_DEV) UPLOAD_PORT=$(UPLOAD_PORT) PROXY_PORT=$(PROXY_PORT) PIO_BUILD_FLAGS="-DSERIAL_MUX_ENABLE=0" scripts/flash_with_restore.sh

.PHONY: flash-mux-no-debug
flash-mux-no-debug:
	@PIO_PROJECT=$(PIO_PROJECT) SERIAL_DEV=$(SERIAL_DEV) UPLOAD_PORT=$(UPLOAD_PORT) PROXY_PORT=$(PROXY_PORT) PIO_BUILD_FLAGS="-DSERIAL_MUX_DEBUG_ENABLE=0" scripts/flash_with_restore.sh

.PHONY: flash-mux-no-debug-skip-ping
flash-mux-no-debug-skip-ping:
	@PIO_PROJECT=$(PIO_PROJECT) SERIAL_DEV=$(SERIAL_DEV) UPLOAD_PORT=$(UPLOAD_PORT) PROXY_PORT=$(PROXY_PORT) PIO_BUILD_FLAGS="-DSERIAL_MUX_DEBUG_ENABLE=0 -DSERIAL_MUX_SKIP_PING=1" scripts/flash_with_restore.sh

.PHONY: ros
ros:
	@if podman ps --format '{{.Names}}' | grep -qx core; then \
		podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 $(ARGS)"; \
	else \
		$(COMPOSE) exec -T core bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 $(ARGS)"; \
	fi

.PHONY: ros-joystick
ros-joystick: podman-core-sync-deps
	@podman rm -f $(PODMAN_JOY_CONTAINER) >/dev/null 2>&1 || true
	@podman commit core $(PODMAN_CORE_RUNTIME_IMAGE) >/dev/null
	@podman run --rm --name $(PODMAN_JOY_CONTAINER) --network $(PODMAN_NETWORK) \
		-v $(CURDIR)/rover:/root/ros2_ws/src/rover:Z \
		-v /dev/input:/dev/input:ro \
		$(PODMAN_CORE_RUNTIME_IMAGE) \
		bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch rover_description joystick.launch.py joy_backend:=$(JOY_BACKEND) joy_dev:=$(JOY_DEV) joy_device_id:=$(JOY_DEVICE_ID)$(if $(JOY_DEVICE_NAME), joy_device_name:='$(JOY_DEVICE_NAME)',)"

.PHONY: ros-stop-joystick
ros-stop-joystick:
	@podman rm -f $(PODMAN_JOY_CONTAINER) >/dev/null 2>&1 || true

.PHONY: ros-echo-joy
ros-echo-joy:
	@if podman ps --format '{{.Names}}' | grep -qx core; then \
		podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic echo /joy"; \
	else \
		$(COMPOSE) exec -T core bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic echo /joy"; \
	fi

.PHONY: ros-echo-cmd-vel
ros-echo-cmd-vel:
	@if podman ps --format '{{.Names}}' | grep -qx core; then \
		podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic echo /cmd_vel"; \
	else \
		$(COMPOSE) exec -T core bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 topic echo /cmd_vel"; \
	fi

.PHONY: core-container-ready
core-container-ready: $(CORE_CONTAINER_STAMP)

$(CORE_CONTAINER_STAMP): $(CORE_IMAGE_INPUTS)
	@mkdir -p $(dir $@)
	@if podman ps --format '{{.Names}}' | grep -qx core; then \
		echo "Using existing podman core container; skipping compose rebuild."; \
	else \
		echo "Rebuilding core container because tracked inputs changed..."; \
		$(COMPOSE) up -d --build core; \
	fi
	@touch $@

.PHONY: podman-core-build
podman-core-build: $(CORE_CONTAINER_STAMP)
	@podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build"

.PHONY: podman-core-sync-deps
podman-core-sync-deps: $(CORE_CONTAINER_STAMP)
	@podman exec core bash -lc "source /opt/ros/jazzy/setup.bash && apt-get update && cd /root/ros2_ws && rosdep install -i --from-path src --rosdistro jazzy -y && colcon build"

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
	@source ~/.platformio/penv/bin/activate && pio run -d $(PIO_PROJECT) -t upload --upload-port $(UPLOAD_PORT)
	@$(MAKE) podman-start-proxy
	@$(MAKE) podman-core-rebuild-msgs

.PHONY: podman-core-rebuild-msgs
podman-core-rebuild-msgs:
	@echo "Rebuilding rover_description in core container..."
	@podman exec core bash -lc 'source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build --packages-select rover_description' 2>&1

.PHONY: e2e
e2e: flash
	@SERIAL_DEV=$(SERIAL_DEV) BAUDRATE=$(BAUDRATE) PROXY_PORT=$(PROXY_PORT) ./scripts/manual_e2e.sh

.PHONY: teleop
teleop: flash
	@SERIAL_DEV=$(SERIAL_DEV) BAUDRATE=$(BAUDRATE) PROXY_PORT=$(PROXY_PORT) ./scripts/manual_teleop.sh

.PHONY: e2e-teleop
e2e-teleop: teleop
