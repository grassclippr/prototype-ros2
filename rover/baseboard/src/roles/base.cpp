#include "./base.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "./espnow.h"

static Basestation *selfBasestation = nullptr;

Basestation::Basestation() {
    selfBasestation = this;

    leds.setup();
    leds.status_led.blink2();

    leds.bootButton.attachClick([]() {
        if (selfBasestation->pairingTaskHandle == nullptr) {
            printf("Starting pairing mode\n");
            selfBasestation->startPairing();
        } else {
            printf("Stopping pairing mode\n");
            selfBasestation->stopPairing();
        }
    });

    // Initialize ESP-NOW
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
            selfBasestation->onEspNowRecv(mac, data, len);
        });
        esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
            if (status != ESP_NOW_SEND_SUCCESS) {
                digitalWrite(ERROR_LED, HIGH);
                delay(30);
                digitalWrite(ERROR_LED, LOW);
            }
        });

        // Make sure to load the stored MAC from NVS
        loadPeerFromNVS();
    }

    // Start GNSS receive task
    xTaskCreate(
        gnssReceiveTask,
        "gnssReceiveTask",
        4000,
        this,
        1,
        nullptr);
};

void Basestation::gnssReceiveTask(void *arg) {
    Basestation *self = (Basestation *)arg;

    // start uart port with UART_RX_PIN and UART_TX_PIN
    Serial2.begin(460800, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    // Serial2.print("$PQTMCFGMSGRATE,W,GGA,1,1*58\r\n");
    // Serial2.print("$PAIR062,0,1*3F\r\n");
    self->sendNmeaCommand("PAIR062,0,1");

    while (true) {
        if (Serial2.available() < 1) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        int first = Serial2.peek();

        switch (first) {
            case 0xD3: {
                // RTCM message
                Serial2.read();  // consume first byte
                int l1 = Serial2.read();
                int l2 = Serial2.read();
                if (l1 < 0 || l2 < 0) {
                    printf("Error reading RTCM length bytes\n");
                    break;
                }
                int length = l1 * 256 + l2;

                if (length <= 0 || length > 1023) {
                    printf("Invalid RTCM length: %d\n", length);
                    // Optionally, flush serial buffer here
                    break;
                }

                // Read RTCM payload
                std::vector<uint8_t> buf(length);
                int read_bytes = Serial2.readBytes(buf.data(), length);
                if (read_bytes != length) {
                    printf("Error reading RTCM payload (%d/%d bytes)\n", read_bytes, length);
                    break;
                }

                // Read checksum (3 bytes)
                uint8_t checksum[3];
                int cs_read = Serial2.readBytes(checksum, 3);
                if (cs_read != 3) {
                    printf("Error reading RTCM checksum\n");
                    break;
                }

                // RTCM message id
                int id = (int(buf[0]) << 4) + (int(buf[1]) >> 4);

                printf("[RTCM] len=%d id=%d\n", length, id);

                // Send it to our peer
                sendRtcmOverEspNow(buf.data(), buf.size());
                break;
            }
            case '$': {
                // NMEA or proprietary message
                String line = Serial2.readStringUntil('\n');
                if (line.length() == 0) {
                    printf("Error reading NMEA line\n");
                    break;
                }

                sendNmeaOverEspNow(line);

                // Split line by ','
                std::vector<String> parts;
                int start = 0;
                int idx = 0;
                while ((idx = line.indexOf(',', start)) != -1) {
                    parts.push_back(line.substring(start, idx));
                    start = idx + 1;
                }
                parts.push_back(line.substring(start));

                if (line.indexOf("ERROR") != -1 && parts.size() >= 3) {
                    printf("[ERROR] command=%s error=%s\n", parts[0].c_str(), parts[2].c_str());
                }
                printf("%s\n", line.c_str());
                break;
            }
            default:
                Serial2.read();  // consume unknown byte
                // printf("Unknown first byte: 0x%02X\n", first);
                break;
        }
        yield();
    }
}

void Basestation::onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len) {
    if (len < 1) {
        return;
    }

    switch (data[0]) {
        case MSG_TYPE_PAIR_ACK:  // Add the rover as a peer
            addPeer(mac_addr);
            paired = true;
            break;
        default:
            digitalWrite(ERROR_LED, HIGH);
            delay(100);
            digitalWrite(ERROR_LED, LOW);
            printf("Unknown ESP-NOW message received: %02X\n", data[0]);
            break;
    }
}

void Basestation::sendNmeaCommand(const String &cmd) {
    // Calculate checksum (XOR of all bytes)
    uint8_t checksum = 0;
    for (size_t i = 0; i < cmd.length(); ++i) {
        checksum ^= cmd[i];
    }
    // Format and send: $<cmd>*<checksum>\r\n
    char buf[128];
    snprintf(buf, sizeof(buf), "$%s*%02X\r\n", cmd.c_str(), checksum);
    Serial2.print(buf);
}

void Basestation::startPairing() {
    stopPairing();

    xTaskCreate(
        pairingTask,
        "pairTask",
        3000,
        this,
        1,
        &pairingTaskHandle);
}

void Basestation::stopPairing() {
    if (pairingTaskHandle != nullptr) {
        vTaskDelete(pairingTaskHandle);
        pairingTaskHandle = nullptr;
    }

    digitalWrite(LYNX_A_LED, LOW);
    digitalWrite(LYNX_B_LED, LOW);
}

void Basestation::pairingTask(void *arg) {
    Basestation *self = (Basestation *)arg;
    self->paired = false;
    uint8_t tries = 0;
    uint8_t ch = 1;

    digitalWrite(LYNX_A_LED, HIGH);
    digitalWrite(LYNX_B_LED, HIGH);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (!self->paired) {
        tries++;
        if (tries > 600) {  // 60 seconds timeout
            printf("Pairing timed out\n");
            break;
        }

        esp_now_deinit();  // Clean up previous state, if any
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

        if (esp_now_init() != ESP_OK) {
            printf("ESP-NOW init failed\n");
            xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
            continue;
        }

        esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
            selfBasestation->onEspNowRecv(mac, data, len);
        });

        if (!addBroadcast()) {
            printf("Failed to add peer for pairing\n");
            break;
        }

        // Broadcast pairing request
        uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        uint8_t pairing_request[] = {MSG_TYPE_PAIR_REQ};
        esp_err_t result = esp_now_send(broadcastAddress, pairing_request, sizeof(pairing_request));
        printf("Sent pairing request on channel %d: ", ch);
        if (result == ESP_OK) {
            printf("Success\n");
        } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
            printf("ESPNOW not Init\n");
        } else if (result == ESP_ERR_ESPNOW_ARG) {
            printf("Invalid Argument\n");
        } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
            printf("Internal Error\n");
        } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
            printf("ESP_ERR_ESPNOW_NO_MEM\n");
        } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
            printf("Peer not found\n");
        } else {
            printf("Not sure what happened\n");
        }

        ch++;
        if (ch > NUM_CHANNELS) {
            ch = 1;  // Wrap around to the first channel
        }

        xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    }

    printf("Pairing task finished\n");
    digitalWrite(LYNX_A_LED, LOW);
    digitalWrite(LYNX_B_LED, LOW);

    self->pairingTaskHandle = nullptr;
    vTaskDelete(NULL);
}
