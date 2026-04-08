#include "./error.h"

#include <cstddef>
#include <cstdint>

namespace {

constexpr size_t kMaxErrorBytes = 96;
constexpr size_t kMaxHexChars = kMaxErrorBytes * 3 + 1;

bool is_printable_ascii(char ch) {
    return ch >= 32 && ch <= 126;
}

void sanitize_error_string(const char *source, char *ascii_out, size_t ascii_size, char *hex_out, size_t hex_size, bool *had_nonprintable) {
    if (ascii_size == 0 || hex_size == 0 || had_nonprintable == nullptr) {
        return;
    }

    ascii_out[0] = '\0';
    hex_out[0] = '\0';
    *had_nonprintable = false;

    if (source == nullptr) {
        return;
    }

    size_t ascii_idx = 0;
    size_t hex_idx = 0;
    for (size_t i = 0; i < kMaxErrorBytes; ++i) {
        unsigned char byte = static_cast<unsigned char>(source[i]);
        if (byte == '\0') {
            break;
        }

        if (ascii_idx + 1 < ascii_size) {
            if (is_printable_ascii(static_cast<char>(byte))) {
                ascii_out[ascii_idx++] = static_cast<char>(byte);
            } else {
                ascii_out[ascii_idx++] = '?';
                *had_nonprintable = true;
            }
        }

        if (hex_idx + 4 < hex_size) {
            int written = snprintf(hex_out + hex_idx, hex_size - hex_idx, "%02X ", byte);
            if (written <= 0) {
                break;
            }
            hex_idx += static_cast<size_t>(written);
        }
    }

    ascii_out[ascii_idx] = '\0';
    if (hex_idx > 0 && hex_idx < hex_size) {
        hex_out[hex_idx - 1] = '\0';
    } else {
        hex_out[0] = '\0';
    }
}

}  // namespace

const char *rcl_ret_to_string(rcl_ret_t rc) {
    switch (rc) {
        case RCL_RET_OK:
            return "RCL_RET_OK";
        case RCL_RET_ERROR:
            return "RCL_RET_ERROR";
        case RCL_RET_TIMEOUT:
            return "RCL_RET_TIMEOUT";
        case RCL_RET_BAD_ALLOC:
            return "RCL_RET_BAD_ALLOC";
        case RCL_RET_INVALID_ARGUMENT:
            return "RCL_RET_INVALID_ARGUMENT";
        case RCL_RET_NODE_INVALID:
            return "RCL_RET_NODE_INVALID";
        case RCL_RET_PUBLISHER_INVALID:
            return "RCL_RET_PUBLISHER_INVALID";
        case RCL_RET_SUBSCRIPTION_INVALID:
            return "RCL_RET_SUBSCRIPTION_INVALID";
        case RCL_RET_TIMER_INVALID:
            return "RCL_RET_TIMER_INVALID";
        case RCL_RET_SERVICE_INVALID:
            return "RCL_RET_SERVICE_INVALID";
        case RCL_RET_CLIENT_INVALID:
            return "RCL_RET_CLIENT_INVALID";
        case RCL_RET_WAIT_SET_INVALID:
            return "RCL_RET_WAIT_SET_INVALID";
        default:
            return "RCL_RET_UNKNOWN";
    }
}

void log_rcl_error(const char *what, rcl_ret_t rc) {
    const rcl_error_string_t error_string = rcl_get_error_string();
    char ascii_error[kMaxErrorBytes + 1];
    char hex_error[kMaxHexChars];
    bool had_nonprintable = false;

    sanitize_error_string(error_string.str, ascii_error, sizeof(ascii_error), hex_error, sizeof(hex_error), &had_nonprintable);

    if (error_string.str == nullptr) {
        printf("%s failed rc=%d (%s): <no error string>\n",
               what,
               static_cast<int>(rc),
               rcl_ret_to_string(rc));
    } else if (had_nonprintable) {
        printf("%s failed rc=%d (%s): ascii=\"%s\" hex=\"%s\"\n",
               what,
               static_cast<int>(rc),
               rcl_ret_to_string(rc),
               ascii_error,
               hex_error);
    } else {
        printf("%s failed rc=%d (%s): %s\n",
               what,
               static_cast<int>(rc),
               rcl_ret_to_string(rc),
               ascii_error);
    }
    rcl_reset_error();
}
