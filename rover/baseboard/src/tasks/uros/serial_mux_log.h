#pragma once

#include "serial_mux.h"

namespace serial_mux {

// Install stdout/printf and esp_log redirection through the mux.
// Must be called after the SerialMux instance is created.
void installLogRedirect(SerialMux *mux);

// Register a shutdown handler that flushes the mux on panic.
void installPanicHandler(SerialMux *mux);

}  // namespace serial_mux
