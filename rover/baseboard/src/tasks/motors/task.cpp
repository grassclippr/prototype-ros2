#include <Arduino.h>
#include "./task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    (void)self;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);
    }
}

void MotorControl::setCommand(float linear_x, float angular_z, uint32_t seq, uint32_t timeout_ms) {
    linear_x_ = linear_x;
    angular_z_ = angular_z;
    seq_ = seq;
    timeout_ms_ = timeout_ms;
    last_update_ms_ = millis();
    active_ = true;
}

void MotorControl::stop() {
    linear_x_ = 0.0f;
    angular_z_ = 0.0f;
    active_ = false;
}

bool MotorControl::expireIfTimedOut(uint32_t now_ms, uint32_t *expired_seq) {
    if (!active_ || timeout_ms_ == 0) {
        return false;
    }

    if (static_cast<uint32_t>(now_ms - last_update_ms_) < timeout_ms_) {
        return false;
    }

    if (expired_seq != nullptr) {
        *expired_seq = seq_;
    }
    stop();
    return true;
}

MotorControl::Command MotorControl::getCommand() const {
    Command command;
    command.linear_x = linear_x_;
    command.angular_z = angular_z_;
    command.seq = seq_;
    command.timeout_ms = timeout_ms_;
    command.last_update_ms = last_update_ms_;
    command.active = active_;
    return command;
}
