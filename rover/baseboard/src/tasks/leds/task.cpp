#include "./task.h"

void LedControl::setup() {
    status_led.begin();
    status_led.off();

    xTaskCreate(
        task,
        "ledTask",
        3000,
        this,
        1,
        NULL);
}

void LedControl::task(void *arg) {
    LedControl *self = static_cast<LedControl *>(arg);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
        self->status_led.loop();
    }
}