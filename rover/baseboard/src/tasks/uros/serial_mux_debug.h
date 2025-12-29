#pragma once

#include <cstddef>
#include <cstdint>

#include "serial_mux.h"

namespace serial_mux {

void set_debug_mux(SerialMux *mux);
int debug_printf(const char *fmt, ...);
void debug_write(const uint8_t *data, size_t len);

}  // namespace serial_mux
