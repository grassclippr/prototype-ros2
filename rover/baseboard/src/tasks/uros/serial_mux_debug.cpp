#include "serial_mux_debug.h"

#include <cstdarg>
#include <cstdio>

#ifndef SERIAL_MUX_ENABLE
#define SERIAL_MUX_ENABLE 1
#endif

#ifndef SERIAL_MUX_DEBUG_ENABLE
#define SERIAL_MUX_DEBUG_ENABLE 1
#endif

namespace serial_mux {

#if SERIAL_MUX_ENABLE && SERIAL_MUX_DEBUG_ENABLE
static SerialMux *g_mux = nullptr;

void set_debug_mux(SerialMux *mux) {
    g_mux = mux;
}

int debug_printf(const char *fmt, ...) {
    if (!g_mux || !fmt) {
        return 0;
    }

    char buffer[512];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (written <= 0) {
        return written;
    }

    size_t len = static_cast<size_t>(written);
    if (len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }

    g_mux->writeDebug(reinterpret_cast<const uint8_t *>(buffer), len);
    return written;
}

void debug_write(const uint8_t *data, size_t len) {
    if (!g_mux || !data || len == 0) {
        return;
    }
    g_mux->writeDebug(data, len);
}
#else
void set_debug_mux(SerialMux *mux) {
    (void)mux;
}

int debug_printf(const char *fmt, ...) {
    (void)fmt;
    return 0;
}

void debug_write(const uint8_t *data, size_t len) {
    (void)data;
    (void)len;
}
#endif

}  // namespace serial_mux
