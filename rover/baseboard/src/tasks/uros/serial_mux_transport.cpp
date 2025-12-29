#include "serial_mux_transport.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

using serial_mux::SerialMux;

extern "C" bool serial_mux_transport_open(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}

extern "C" bool serial_mux_transport_close(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}

extern "C" size_t serial_mux_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)err;
    if (!transport || !transport->args || !buf || len == 0) {
        return 0;
    }

    SerialMux *mux = static_cast<SerialMux *>(transport->args);
    return mux->writeRos(buf, len);
}

extern "C" size_t serial_mux_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    (void)err;
    if (!transport || !transport->args || !buf || len == 0) {
        return 0;
    }

    SerialMux *mux = static_cast<SerialMux *>(transport->args);
    return mux->readRos(buf, len, timeout);
}
