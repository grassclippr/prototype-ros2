#include "./espnow.h"
#include <Preferences.h>

Preferences nvs;

void addEspNowPeer(const uint8_t *mac) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (!esp_now_is_peer_exist(mac)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            printf("Failed to add peer\n");
        } else {
            nvs.begin("espnow", false); // Open NVS namespace "espnow"
            nvs.putBytes(NVS_PEER_KEY, mac, 6); // Store 6 bytes (MAC address)
            nvs.end();

            printf("Peer added\n");
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
    nvs.end();
    
    if (len != 6) {
        return false;
    }

    if (!esp_now_is_peer_exist(peerInfo.peer_addr)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            return false;
        }
    }

    return true;
}
