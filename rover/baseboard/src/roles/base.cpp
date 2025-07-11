#include <Arduino.h>
#include "./base.h"
#include "./espnow.h"
#include <WiFi.h>
#include <esp_wifi.h>

static Basestation * selfBasestation = nullptr;

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
            if (status == ESP_NOW_SEND_SUCCESS) {
                digitalWrite(ERROR_LED, HIGH);
                delay(30);
                digitalWrite(ERROR_LED, LOW);
            } else {
                digitalWrite(ERROR_LED, HIGH);
                delay(400);
                digitalWrite(ERROR_LED, LOW);
            }
        });
        
        // Make sure to load the stored MAC from NVS
        loadPeerFromNVS();
    }
};

void Basestation::onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len) {
    if (len < 1) {
        return;
    }

    switch(data[0]) {
        case MSG_TYPE_PAIR_ACK: // Add the rover as a peer
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
        if (tries > 600) { // 60 seconds timeout
            printf("Pairing timed out\n");
            break;
        }

        esp_now_deinit(); // Clean up previous state, if any
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
        uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        uint8_t pairing_request[] = { MSG_TYPE_PAIR_REQ };
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
            ch = 1; // Wrap around to the first channel
        }

        xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    }

    printf("Pairing task finished\n");
    digitalWrite(LYNX_A_LED, LOW);
    digitalWrite(LYNX_B_LED, LOW);

    self->pairingTaskHandle = nullptr;
    vTaskDelete(NULL);
}
