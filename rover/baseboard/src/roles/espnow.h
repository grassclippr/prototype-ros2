#pragma once
#include <Arduino.h>
#include <esp_now.h>

#define NVS_PEER_KEY "peer_mac"
#define NVS_PEER_CHANNEL "peer_channel"
#define NUM_CHANNELS 13  // 13 in europe, 14 in japan, 11 in the US, 12 in China
#define RTCM_ESP_NOW_MAX_PAYLOAD 250

enum EspNowMsgType {
    MSG_TYPE_UNKNOWN = 0x00,
    MSG_TYPE_FIRMWARE_REQ = 0x01,
    MSG_TYPE_FIRMWARE_RES = 0x02,
    MSG_TYPE_RTCM = 0x03,
    MSG_TYPE_NMEA = 0x04,
    MSG_TYPE_PAIR_REQ = 0xAA,
    MSG_TYPE_PAIR_ACK = 0xAB,
};

void addPeer(const uint8_t* mac);
bool loadPeerFromNVS();

bool addBroadcast();
bool removeBroadcast();

void sendRtcmOverEspNow(const uint8_t* data, size_t len);
void sendNmeaOverEspNow(const String& nmea);
