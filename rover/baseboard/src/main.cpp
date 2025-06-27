#include <Arduino.h>

#include "tasks/leds/task.h"
#include "tasks/motors/task.h"
#include "tasks/uros/client.h"

UrosClient uros_client;
LedControl leds;
MotorControl motors;

void setup() {
    leds.setup();
    motors.setup();
    uros_client.setup();

    uros_client.subscribeToStateChange([](ClientState state) {
        if (state == AGENT_CONNECTED) {
            leds.status_led.on();
        } else {
            leds.status_led.blink1();
        }
    });
}

void loop() {
    vTaskDelete(NULL);
}