#include <Arduino.h>
#include "./task.h"

void LedControl::setup() {
    pinMode(ERROR_LED, OUTPUT);
    pinMode(TWAI_LED, OUTPUT);
    pinMode(LYNX_A_LED, OUTPUT);
    pinMode(LYNX_B_LED, OUTPUT);
    pinMode(LYNX_C_LED, OUTPUT);
    pinMode(LYNX_D_LED, OUTPUT);


    // Initialize buttons
    bootButton.setup(BOOT_BUTTON_PIN);

    // Flash ERROR to indicate boot
    digitalWrite(STATUS_LED, HIGH);
    digitalWrite(ERROR_LED, HIGH);
    digitalWrite(TWAI_LED, HIGH);
    digitalWrite(LYNX_A_LED, HIGH);
    digitalWrite(LYNX_B_LED, HIGH);
    digitalWrite(LYNX_C_LED, HIGH);
    digitalWrite(LYNX_D_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(ERROR_LED, LOW);
    digitalWrite(TWAI_LED, LOW);
    digitalWrite(LYNX_A_LED, LOW);
    digitalWrite(LYNX_B_LED, LOW);
    digitalWrite(LYNX_C_LED, LOW);
    digitalWrite(LYNX_D_LED, LOW);

    // Initialize leds controlled by BlinkControl lib
    status_led.begin();
    status_led.fastBlinking();

    xTaskCreate(
        task,
        "ledTask",
        4000,
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
