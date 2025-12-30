PIO_PROJECT := rover/baseboard

.PHONY: flash
flash:
	@. ~/.platformio/penv/bin/activate && \
		pio run -t upload -d $(PIO_PROJECT)

.PHONY: flash-no-ros
flash-no-ros:
	@. ~/.platformio/penv/bin/activate && \
		PIO_BUILD_FLAGS="-DSERIAL_MUX_DISABLE_ROS=1" pio run -t upload -d $(PIO_PROJECT)

.PHONY: flash-no-mux
flash-no-mux:
	@. ~/.platformio/penv/bin/activate && \
		PIO_BUILD_FLAGS="-DSERIAL_MUX_ENABLE=0" pio run -t upload -d $(PIO_PROJECT)

.PHONY: flash-mux-no-debug
flash-mux-no-debug:
	@. ~/.platformio/penv/bin/activate && \
		PIO_BUILD_FLAGS="-DSERIAL_MUX_DEBUG_ENABLE=0" pio run -t upload -d $(PIO_PROJECT)

.PHONY: flash-mux-no-debug-skip-ping
flash-mux-no-debug-skip-ping:
	@. ~/.platformio/penv/bin/activate && \
		PIO_BUILD_FLAGS="-DSERIAL_MUX_DEBUG_ENABLE=0 -DSERIAL_MUX_SKIP_PING=1" pio run -t upload -d $(PIO_PROJECT)
