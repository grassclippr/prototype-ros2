#include "./task.h"

#include <Arduino.h>

void MotorControl::setup() {
    xTaskCreate(
        task,
        "motorTask",
        3000,
        this,
        1,
        NULL);
}

void MotorControl::task(void *arg) {
    MotorControl *self = static_cast<MotorControl *>(arg);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
    }
}