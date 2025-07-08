#include "./base.h"
#include "./espnow.h"

static Basestation * selfBasestation = nullptr;

Basestation::Basestation() {
    selfBasestation = this;

    leds.setup();

    leds.status_led.blink2();

    // Initialize ESP-NOW
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
            selfBasestation->onEspNowRecv(mac, data, len);
        });
        //esp_now_register_send_cb(onEspNowSent);
        
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
            addEspNowPeer(mac_addr);
            break;
        default:
            printf("Unknown ESP-NOW message received: %02X\n", data[0]);
            break;
    }
}
