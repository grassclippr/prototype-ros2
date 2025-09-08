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

    size_t write(uint8_t byte) override {
        base_stream.print("sending 1 byte\n");

        // Apply COBS encoding
        uint8_t encoded[6];  // 0x00, 0x00, data..., 0x00
        encoded[0] = 0x00;   // Start marker
        encoded[1] = 0x00;   // Start marker
        size_t len = cobs_encode(&byte, 1, encoded + 2);
        encoded[len + 2] = 0x00;  // End marker
        return base_stream.write(encoded, len + 3);
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        // Apply COBS encoding
        uint8_t encoded[size + size / 254 + 3 + 3];  // Worst case size
        encoded[0] = 0x00;                           // Start marker
        encoded[1] = 0x00;                           // Start marker
        size_t len = cobs_encode(buffer, size, encoded + 2);
        encoded[len + 2] = 0x00;  // End marker

        // Write the data in 32 byte chunks to avoid overwhelming the USB buffer
        size_t n = 0;
        for (size_t i = 0; i < len + 3; i += 32) {
            size_t chunk_size = std::min<size_t>(32, len + 3 - i);
            base_stream.write(encoded + i, chunk_size);
            base_stream.flush();
            n += chunk_size;
        }

        base_stream.printf("sending %d (%d) bytes\n", size, len+3);

        return n;
    }

    // Implement other Stream methods as needed...

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
