#pragma once
#include <Arduino.h>

#include "tasks/leds/task.h"

class Basestation {
    public:
        Basestation();

    private:
        // Tasks
        LedControl leds;

        // ESP-NOW communication
        void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);
};
