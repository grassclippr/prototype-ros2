#pragma once
#include <Arduino.h>
#include <esp_now.h>

#define NVS_PEER_KEY "esp-now_peer_mac"

enum EspNowMsgType {
    MSG_TYPE_UNKNOWN = 0x00,
    MSG_TYPE_FIRMWARE_REQ = 0x01,
    MSG_TYPE_FIRMWARE_RES = 0x02,
    MSG_TYPE_RTCM = 0x03,
    MSG_TYPE_PAIR_REQ = 0xAA,
    MSG_TYPE_PAIR_ACK = 0xAB,
};

void addEspNowPeer(const uint8_t *mac);
bool loadPeerFromNVS();
