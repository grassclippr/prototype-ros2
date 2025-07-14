#include "./espnow.h"
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>

extern Preferences nvs;
uint8_t peer_mac[6];

void addPeer(const uint8_t *mac) {
    if (mac == nullptr) {
        printf("MAC address is null\n");
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = WiFi.channel();
    peerInfo.encrypt = false;

    if (!esp_now_is_peer_exist(mac)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            printf("Failed to add peer\n");
        } else {
            nvs.begin("espnow", false); // Open NVS namespace "espnow"
            nvs.putBytes(NVS_PEER_KEY, mac, 6); // Store 6 bytes (MAC address)
            nvs.putUChar(NVS_PEER_CHANNEL, peerInfo.channel); // Store channel
            nvs.end();

            printf("Peer %02x:%02x:%02x:%02x:%02x:%02x added on channel %d\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],peerInfo.channel);
        }
    }
}

// Load a peer MAC from NVS, returns true if found
bool loadPeerFromNVS() {
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    nvs.begin("espnow", true); // Read-only
    size_t len = nvs.getBytes(NVS_PEER_KEY, peerInfo.peer_addr, 6);
    peerInfo.channel = nvs.getUChar(NVS_PEER_CHANNEL);
    nvs.end();
    
    if (len != 6) {
        return false;
    }

    if (!esp_now_is_peer_exist(peerInfo.peer_addr)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            return false;
        }
    }

    peer_mac[0] = peerInfo.peer_addr[0];
    peer_mac[1] = peerInfo.peer_addr[1];
    peer_mac[2] = peerInfo.peer_addr[2];
    peer_mac[3] = peerInfo.peer_addr[3];
    peer_mac[4] = peerInfo.peer_addr[4];
    peer_mac[5] = peerInfo.peer_addr[5];

    esp_wifi_set_channel(peerInfo.channel, WIFI_SECOND_CHAN_NONE);

    return true;
}

bool addBroadcast() {
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    //peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false;
    peerInfo.peer_addr[0] = 0xFF;
    peerInfo.peer_addr[1] = 0xFF;
    peerInfo.peer_addr[2] = 0xFF;
    peerInfo.peer_addr[3] = 0xFF;
    peerInfo.peer_addr[4] = 0xFF;
    peerInfo.peer_addr[5] = 0xFF;

    if (!esp_now_is_peer_exist(peerInfo.peer_addr)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            return false;
        }
    }

    return true;
}

bool removeBroadcast() {
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.peer_addr[0] = 0xFF;
    peerInfo.peer_addr[1] = 0xFF;
    peerInfo.peer_addr[2] = 0xFF;
    peerInfo.peer_addr[3] = 0xFF;
    peerInfo.peer_addr[4] = 0xFF;
    peerInfo.peer_addr[5] = 0xFF;

    if (esp_now_is_peer_exist(peerInfo.peer_addr)) {
        esp_now_del_peer(peerInfo.peer_addr);
    }

    return true;
}

void sendRtcmOverEspNow(const uint8_t* data, size_t len) {
    size_t max_data_per_packet = RTCM_ESP_NOW_MAX_PAYLOAD - 3;
    uint8_t total_parts = (len + max_data_per_packet - 1) / max_data_per_packet;

    for (uint8_t part_idx = 0; part_idx < total_parts; ++part_idx) {
        size_t offset = part_idx * max_data_per_packet;
        size_t chunk_len = std::min(max_data_per_packet, len - offset);

        uint8_t packet[RTCM_ESP_NOW_MAX_PAYLOAD];
        packet[0] = MSG_TYPE_RTCM;
        packet[1] = total_parts;
        packet[2] = part_idx;
        memcpy(packet + 3, data + offset, chunk_len);

        esp_now_send(peer_mac, packet, chunk_len + 3);
    }
}

void sendNmeaOverEspNow(const String& nmea) {
    const uint8_t* data = (const uint8_t*)nmea.c_str();
    size_t len = nmea.length();
    size_t max_data_per_packet = RTCM_ESP_NOW_MAX_PAYLOAD - 3;
    uint8_t total_parts = (len + max_data_per_packet - 1) / max_data_per_packet;

    for (uint8_t part_idx = 0; part_idx < total_parts; ++part_idx) {
        size_t offset = part_idx * max_data_per_packet;
        size_t chunk_len = std::min(max_data_per_packet, len - offset);

        uint8_t packet[RTCM_ESP_NOW_MAX_PAYLOAD];
        packet[0] = MSG_TYPE_NMEA; // Define this as a new message type, e.g. #define MSG_TYPE_NMEA 0x03
        packet[1] = total_parts;
        packet[2] = part_idx;
        memcpy(packet + 3, data + offset, chunk_len);

        esp_now_send(peer_mac, packet, chunk_len + 3);
    }
}