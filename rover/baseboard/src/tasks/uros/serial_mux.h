#pragma once

#include <Arduino.h>
#include <Stream.h>

#include <cstddef>
#include <cstdint>

#include "freertos/semphr.h"

namespace serial_mux {

static constexpr uint8_t kEnd = 0xC0;
static constexpr uint8_t kEsc = 0xDB;
static constexpr uint8_t kEscEnd = 0xDC;
static constexpr uint8_t kEscEsc = 0xDD;

static constexpr uint8_t kMagic = 0xA7;
static constexpr uint8_t kVersion = 0x01;

static constexpr uint8_t kTypeRos = 0x01;
static constexpr uint8_t kTypeDebug = 0x02;

static constexpr uint8_t kFlagChunked = 0x01;
static constexpr uint8_t kFlagChunkStart = 0x02;
static constexpr uint8_t kFlagChunkEnd = 0x04;

static constexpr size_t kMaxPayload = 256;
static constexpr size_t kMaxRosMessageLen = 4096;

class SerialMux {
   public:
    explicit SerialMux(Stream &stream);

    size_t writeRos(const uint8_t *data, size_t len);
    size_t readRos(uint8_t *data, size_t len, int timeout_ms);

    void writeDebug(const uint8_t *data, size_t len);
    int writeDebugf(const char *fmt, ...);

   private:
    Stream &stream_;
    uint16_t seq_ = 0;
    uint16_t debug_msg_id_ = 0;
    uint16_t ros_msg_id_ = 0;
    unsigned long frame_start_ms_ = 0;

    static constexpr unsigned long kFrameTimeoutMs = 200;

    uint8_t ros_buffer_[kMaxRosMessageLen] = {};
    size_t ros_len_ = 0;
    size_t ros_offset_ = 0;
    uint8_t ros_reassembly_buffer_[kMaxRosMessageLen] = {};
    size_t ros_reassembly_len_ = 0;
    uint16_t ros_reassembly_msg_id_ = 0;
    bool ros_reassembly_active_ = false;

    static constexpr size_t kHeaderLen = 10;
    static constexpr size_t kCrcLen = 4;
    static constexpr size_t kMaxFrameLen = kHeaderLen + kMaxPayload + kCrcLen;

    uint8_t frame_buf_[kMaxFrameLen];
    size_t frame_len_ = 0;
    bool escape_ = false;
    bool drop_frame_ = false;
    SemaphoreHandle_t write_mutex_ = nullptr;

    void writeFrame(uint8_t frame_type, uint8_t flags, uint16_t msg_id, const uint8_t *payload, size_t len);

    bool fillRosFromSerial(int timeout_ms);
    void resetFrame();
    bool appendDecoded(uint8_t byte);
    bool processFrame();

    uint32_t crc32(const uint8_t *data, size_t len);
};

}  // namespace serial_mux
