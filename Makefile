PIO_PROJECT := rover/baseboard

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

.PHONY: test
test:
	@PYTHONPATH=. uv run pytest

.PHONY: e2e
e2e: flash
	@./scripts/manual_e2e.sh
