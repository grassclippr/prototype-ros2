#include <Arduino.h>
#include "./task.h"

void LedControl::setup() {
    status_led.begin();
    status_led.off();

    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(ERROR_LED, HIGH);
    delay(100);
    digitalWrite(ERROR_LED, LOW);

    button.setup(PIN_INPUT, INPUT_PULLUP, true);

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
        self->button.tick();
    }
}
