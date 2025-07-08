#include <Arduino.h>
#include "./task.h"

void LedControl::setup() {
    pinMode(ERROR_LED, OUTPUT);
    pinMode(TWAI_LED, OUTPUT);
    pinMode(LYNX_A_LED, OUTPUT);
    pinMode(LYNX_B_LED, OUTPUT);
    pinMode(LYNX_C_LED, OUTPUT);
    pinMode(LYNX_D_LED, OUTPUT);

    // Initialize leds controlled by BlinkControl lib
    status_led.begin();

    // Initialize buttons
    bootButton.setup(BOOT_BUTTON_PIN);

    // Flash ERROR to indicate boot
    digitalWrite(ERROR_LED, HIGH);
    delay(100);
    digitalWrite(ERROR_LED, LOW);

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
        xTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
        self->status_led.loop();
        self->bootButton.tick();
    }
}
