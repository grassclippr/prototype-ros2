#include <Arduino.h>
#include <WiFi.h>

#include "roles/roles.h"

DeviceRole device_role = ROLE_UNKNOWN;

Basestation* basestation = nullptr;
Rover* rover = nullptr;

void setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    switch (device_role) {
          case ROLE_BASESTATION:
            basestation = new Basestation();
            break;
        case ROLE_ROVER:
            rover = new Rover();
            break;
        default:
            Serial.println("Unknown device role, cannot initialize tasks.");
            break;
    }
}

void loop() {
    vTaskDelete(NULL);
}
