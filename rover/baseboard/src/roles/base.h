#pragma once
#include <Arduino.h>

#include "tasks/leds/task.h"

class Basestation {
   public:
    Basestation();
    void startPairing();
    void stopPairing();
    static void pairingTask(void *arg);

   private:
    // Tasks
    LedControl leds;
    static void gnssReceiveTask(void *arg);

    void sendNmeaCommand(const String &cmd);

    // ESP-NOW communication
    void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);

    TaskHandle_t pairingTaskHandle = nullptr;
    bool paired = false;
};
