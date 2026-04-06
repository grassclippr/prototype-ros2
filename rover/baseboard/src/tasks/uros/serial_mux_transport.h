#pragma once

#include <cstddef>
#include <cstdint>

#include "serial_mux.h"

extern "C" {

struct uxrCustomTransport;

bool serial_mux_transport_open(struct uxrCustomTransport *transport);
bool serial_mux_transport_close(struct uxrCustomTransport *transport);
size_t serial_mux_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t serial_mux_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

}  // extern "C"
