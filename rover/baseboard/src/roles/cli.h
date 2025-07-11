#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include "tasks/leds/task.h"

extern Preferences nvs;

class CLI {
    public:
        CLI();

    private:
        // Tasks
        LedControl leds;
        static void cliTask(void *arg) {
            CLI *self = static_cast<CLI *>(arg);
            self->loop();
        }
        void loop();
};
