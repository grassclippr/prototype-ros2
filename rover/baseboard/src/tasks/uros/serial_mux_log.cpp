#include "serial_mux_log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include <esp_log.h>
#include <esp_system.h>

namespace serial_mux {

static SerialMux *g_mux = nullptr;

static int mux_vprintf(const char *fmt, va_list args) {
    if (!g_mux) {
        return 0;
    }

    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len <= 0) {
        return 0;
    }
    if (len >= static_cast<int>(sizeof(buf))) {
        len = sizeof(buf) - 1;
    }

    g_mux->writeDebug(reinterpret_cast<const uint8_t *>(buf), static_cast<size_t>(len));
    return len;
}

// Override the newlib _write_r syscall to capture printf/stdout/stderr output.
// On ESP32-S3 with Arduino framework, printf ultimately calls _write_r for
// file descriptors 1 (stdout) and 2 (stderr).
extern "C" int __real__write_r(struct _reent *r, int fd, const void *data, int size);
extern "C" int __wrap__write_r(struct _reent *r, int fd, const void *data, int size) {
    if (g_mux && (fd == 1 || fd == 2) && data && size > 0) {
        g_mux->writeDebug(static_cast<const uint8_t *>(data), static_cast<size_t>(size));
        return size;
    }
    return __real__write_r(r, fd, data, size);
}

void installLogRedirect(SerialMux *mux) {
    g_mux = mux;

    // Redirect ESP-IDF logging (esp_log_write / ESP_LOGx macros)
    esp_log_set_vprintf(mux_vprintf);
}

void installPanicHandler(SerialMux *mux) {
    g_mux = mux;

    esp_register_shutdown_handler([]() {
        if (g_mux) {
            g_mux->panicFlush();
            // Disable redirection so panic handler output goes raw to UART
            // (which the proxy will pick up via resync on next END byte)
            g_mux = nullptr;
        }
    });
}

}  // namespace serial_mux
