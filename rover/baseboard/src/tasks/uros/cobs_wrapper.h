#pragma once
#include <Arduino.h>

#include <algorithm>

class CobsStream : public Stream {
   public:
    CobsStream(Stream &base) : base_stream(base) {}

    int available() override {
        return base_stream.available();
    }

    int read() override {
        return base_stream.read();
    }

    int peek() override {
        return base_stream.peek();
    }

    size_t write(uint8_t data) override {
        return base_stream.write(data);
    }

    /*size_t write(const uint8_t *buffer, size_t size) override {
        // Apply COBS encoding
        uint8_t encoded[2 + size + size / 254 + 1];  // Worst case size
        encoded[0] = 0x00;                           // Start marker
        encoded[1] = 0x00;                           // Start marker
        size_t len = cobs_encode(buffer, size, encoded + 2);
        encoded[len + 2] = 0x00;  // End marker

        base_stream.write(encoded, len + 3);

        return size;
    }*/

    size_t write(const uint8_t *buffer, size_t size) override {
        // worst-case COBS expansion: size + floor(size/254) + 1
        size_t out_payload_max = size + size / 254 + 1;
        // total frame: 2 start markers + payload + 1 end marker
        std::vector<uint8_t> encoded;
        encoded.resize(2 + out_payload_max + 1);

        // start markers
        encoded[0] = 0x00;
        encoded[1] = 0x00;

        // encode into encoded.data() + 2
        size_t len = cobs_encode(buffer, size, encoded.data() + 2);

        // place end marker immediately after encoded payload
        encoded[2 + len] = 0x00;
        // resize to actual frame length
        encoded.resize(2 + len + 1);

        // write to underlying stream and return actual written bytes
        base_stream.write(encoded.data(), encoded.size());
        base_stream.flush();

        return size;
    }

   private:
    Stream &base_stream;

    size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output) {
        size_t read_index = 0;
        size_t write_index = 1;
        size_t code_index = 0;
        uint8_t code = 1;
        while (read_index < length) {
            if (input[read_index] == 0) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
                read_index++;
            } else {
                output[write_index++] = input[read_index++];
                code++;
                if (code == 0xFF) {
                    output[code_index] = code;
                    code = 1;
                    code_index = write_index++;
                }
            }
        }
        output[code_index] = code;
        return write_index;
    };
};
