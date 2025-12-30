#include "serial_mux.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace serial_mux {

SerialMux::SerialMux(Stream &stream) : stream_(stream) {
    write_mutex_ = xSemaphoreCreateMutex();
}

uint32_t SerialMux::crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void SerialMux::writeFrame(uint8_t frame_type, uint8_t flags, uint16_t msg_id, const uint8_t *payload, size_t len) {
    if (len > kMaxPayload) {
        return;
    }

    if (write_mutex_ != nullptr) {
        xSemaphoreTake(write_mutex_, portMAX_DELAY);
    }

    uint8_t raw[kMaxFrameLen];
    size_t idx = 0;
    raw[idx++] = kMagic;
    raw[idx++] = kVersion;
    raw[idx++] = frame_type;
    raw[idx++] = flags;
    raw[idx++] = static_cast<uint8_t>(seq_ & 0xFF);
    raw[idx++] = static_cast<uint8_t>((seq_ >> 8) & 0xFF);
    raw[idx++] = static_cast<uint8_t>(msg_id & 0xFF);
    raw[idx++] = static_cast<uint8_t>((msg_id >> 8) & 0xFF);
    raw[idx++] = static_cast<uint8_t>(len & 0xFF);
    raw[idx++] = static_cast<uint8_t>((len >> 8) & 0xFF);

    if (len > 0) {
        memcpy(raw + idx, payload, len);
        idx += len;
    }

    uint32_t crc = crc32(raw + 1, idx - 1);
    raw[idx++] = static_cast<uint8_t>(crc & 0xFF);
    raw[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    raw[idx++] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    raw[idx++] = static_cast<uint8_t>((crc >> 24) & 0xFF);

    seq_ = static_cast<uint16_t>(seq_ + 1u);

    for (size_t i = 0; i < idx; ++i) {
        uint8_t b = raw[i];
        if (b == kEnd) {
            stream_.write(kEsc);
            stream_.write(kEscEnd);
        } else if (b == kEsc) {
            stream_.write(kEsc);
            stream_.write(kEscEsc);
        } else {
            stream_.write(b);
        }
    }
    stream_.write(kEnd);

    if (write_mutex_ != nullptr) {
        xSemaphoreGive(write_mutex_);
    }
}

size_t SerialMux::writeRos(const uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return 0;
    }

    uint16_t msg_id = 0;
    if (len > kMaxPayload) {
        msg_id = static_cast<uint16_t>(ros_msg_id_ + 1u);
        ros_msg_id_ = msg_id;
    }

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > kMaxPayload) {
            chunk = kMaxPayload;
        }
        uint8_t flags = 0;
        if (len > kMaxPayload) {
            flags |= kFlagChunked;
            if (offset == 0) {
                flags |= kFlagChunkStart;
            }
            if (offset + chunk >= len) {
                flags |= kFlagChunkEnd;
            }
        }
        writeFrame(kTypeRos, flags, msg_id, data + offset, chunk);
        offset += chunk;
    }
    return len;
}

void SerialMux::writeDebug(const uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return;
    }

    uint16_t msg_id = static_cast<uint16_t>(debug_msg_id_ + 1u);
    debug_msg_id_ = msg_id;

    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > kMaxPayload) {
            chunk = kMaxPayload;
        }
        uint8_t flags = 0;
        if (len > kMaxPayload) {
            flags |= kFlagChunked;
            if (offset == 0) {
                flags |= kFlagChunkStart;
            }
            if (offset + chunk >= len) {
                flags |= kFlagChunkEnd;
            }
        }
        writeFrame(kTypeDebug, flags, msg_id, data + offset, chunk);
        offset += chunk;
    }
}

int SerialMux::writeDebugf(const char *fmt, ...) {
    if (!fmt) {
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

    writeDebug(reinterpret_cast<const uint8_t *>(buffer), len);
    return written;
}

size_t SerialMux::readRos(uint8_t *data, size_t len, int timeout_ms) {
    if (!data || len == 0) {
        return 0;
    }

    size_t copied = 0;
    unsigned long start = millis();

    while (copied < len) {
        if (ros_offset_ < ros_len_) {
            size_t available = ros_len_ - ros_offset_;
            size_t to_copy = len - copied;
            if (to_copy > available) {
                to_copy = available;
            }
            memcpy(data + copied, ros_buffer_ + ros_offset_, to_copy);
            ros_offset_ += to_copy;
            copied += to_copy;
            break;
        }

        if (!fillRosFromSerial(timeout_ms)) {
            break;
        }

        if (timeout_ms >= 0 && millis() - start >= static_cast<unsigned long>(timeout_ms)) {
            break;
        }
    }

    return copied;
}

bool SerialMux::fillRosFromSerial(int timeout_ms) {
    unsigned long start = millis();
    while (timeout_ms < 0 || millis() - start < static_cast<unsigned long>(timeout_ms)) {
        if (!stream_.available()) {
            delay(1);
            continue;
        }

        int raw = stream_.read();
        if (raw < 0) {
            continue;
        }
        uint8_t byte = static_cast<uint8_t>(raw);

        if (byte == kEnd) {
            if (frame_len_ == 0) {
                resetFrame();
                continue;
            }

            bool got_ros = (!drop_frame_) && processFrame();
            resetFrame();
            if (got_ros) {
                return true;
            }
            continue;
        }

        if (drop_frame_) {
            continue;
        }

        if (frame_len_ == 0 && frame_start_ms_ == 0) {
            frame_start_ms_ = millis();
        }

        if (escape_) {
            escape_ = false;
            if (byte == kEscEnd) {
                byte = kEnd;
            } else if (byte == kEscEsc) {
                byte = kEsc;
            } else {
                drop_frame_ = true;
                continue;
            }
            appendDecoded(byte);
            continue;
        }

        if (byte == kEsc) {
            escape_ = true;
            continue;
        }

        appendDecoded(byte);
    }

    return false;
}

void SerialMux::resetFrame() {
    frame_len_ = 0;
    escape_ = false;
    drop_frame_ = false;
    frame_start_ms_ = 0;
}

bool SerialMux::appendDecoded(uint8_t byte) {
    if (frame_start_ms_ > 0 && millis() - frame_start_ms_ > kFrameTimeoutMs) {
        drop_frame_ = true;
        return false;
    }
    if (frame_len_ >= kMaxFrameLen) {
        drop_frame_ = true;
        return false;
    }
    frame_buf_[frame_len_++] = byte;
    return true;
}

bool SerialMux::processFrame() {
    if (frame_len_ < kHeaderLen + kCrcLen) {
        return false;
    }

    if (frame_buf_[0] != kMagic || frame_buf_[1] != kVersion) {
        return false;
    }

    uint8_t frame_type = frame_buf_[2];
    uint8_t flags = frame_buf_[3];
    uint16_t msg_id = static_cast<uint16_t>(frame_buf_[6]) | (static_cast<uint16_t>(frame_buf_[7]) << 8);
    uint16_t length = static_cast<uint16_t>(frame_buf_[8]) | (static_cast<uint16_t>(frame_buf_[9]) << 8);

    if (length > kMaxPayload) {
        return false;
    }

    size_t expected_len = kHeaderLen + length + kCrcLen;
    if (frame_len_ != expected_len) {
        return false;
    }

    uint32_t crc_read = 0;
    size_t crc_offset = kHeaderLen + length;
    crc_read |= static_cast<uint32_t>(frame_buf_[crc_offset]);
    crc_read |= static_cast<uint32_t>(frame_buf_[crc_offset + 1]) << 8;
    crc_read |= static_cast<uint32_t>(frame_buf_[crc_offset + 2]) << 16;
    crc_read |= static_cast<uint32_t>(frame_buf_[crc_offset + 3]) << 24;

    uint32_t crc_calc = crc32(frame_buf_ + 1, kHeaderLen - 1 + length);
    if (crc_calc != crc_read) {
        return false;
    }

    if (frame_type != kTypeRos) {
        return false;
    }

    const uint8_t *payload = frame_buf_ + kHeaderLen;
    if ((flags & kFlagChunked) == 0) {
        if (length > 0) {
            memcpy(ros_buffer_, payload, length);
        }
        ros_len_ = length;
        ros_offset_ = 0;
        return true;
    }

    if ((flags & kFlagChunkStart) != 0) {
        ros_reassembly_active_ = true;
        ros_reassembly_msg_id_ = msg_id;
        ros_reassembly_len_ = 0;
    }

    if (!ros_reassembly_active_ || ros_reassembly_msg_id_ != msg_id) {
        return false;
    }

    if (ros_reassembly_len_ + length > kMaxRosMessageLen) {
        ros_reassembly_active_ = false;
        ros_reassembly_len_ = 0;
        return false;
    }

    if (length > 0) {
        memcpy(ros_reassembly_buffer_ + ros_reassembly_len_, payload, length);
        ros_reassembly_len_ += length;
    }

    if ((flags & kFlagChunkEnd) == 0) {
        return false;
    }

    if (ros_reassembly_len_ > 0) {
        memcpy(ros_buffer_, ros_reassembly_buffer_, ros_reassembly_len_);
    }
    ros_len_ = ros_reassembly_len_;
    ros_offset_ = 0;
    ros_reassembly_active_ = false;
    ros_reassembly_len_ = 0;
    return true;
}

}  // namespace serial_mux
