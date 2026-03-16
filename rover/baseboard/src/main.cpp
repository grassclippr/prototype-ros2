#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>

#include "roles/roles.h"

DeviceRole device_role = ROLE_UNKNOWN;
Preferences nvs;

CLI* cli = nullptr;
Basestation* basestation = nullptr;
Rover* rover = nullptr;

#ifdef E2E_CRASH_TEST
void crashTestTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("DEBUG: Initiating deliberate crash...\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    *((volatile int*)0) = 0; // Trigger StoreProhibited
    vTaskDelete(NULL);
}
#endif

void setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

#ifdef E2E_CRASH_TEST
    xTaskCreate(crashTestTask, "CrashTest", 2048, NULL, 1, NULL);
#endif

    nvs.begin("core", false);
    device_role = static_cast<DeviceRole>(nvs.getInt("role", static_cast<int>(ROLE_UNKNOWN)));
    nvs.end();

    switch (device_role) {
          case ROLE_BASESTATION:
            basestation = new Basestation();
            break;
        case ROLE_ROVER:
            rover = new Rover();
            break;
        default:
            cli = new CLI();
            break;
    }
}

void loop() {
    vTaskDelete(NULL);
}
