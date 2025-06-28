#include <Arduino.h>

#include "tasks/leds/task.h"
#include "tasks/motors/task.h"
#include "tasks/uros/client.h"

UrosClient uros_client;
LedControl leds;
MotorControl motors;

void setup() {
    USBSerial.begin(115200);

    pinMode(LYNX_A_LED, OUTPUT);
    pinMode(LYNX_B_LED, OUTPUT);
    pinMode(LYNX_C_LED, OUTPUT);
    pinMode(LYNX_D_LED, OUTPUT);

    leds.setup();
    motors.setup();
    uros_client.setup(USBSerial);

    uros_client.subscribeToStateChange([](ClientState state) {
        if (state == AGENT_CONNECTED) {
            leds.status_led.on();
        } else {
            leds.status_led.blink1();
        }

        digitalWrite(LYNX_A_LED, state == WAITING_AGENT ? HIGH : LOW);
        digitalWrite(LYNX_B_LED, state == CONNECTING ? HIGH : LOW);
        digitalWrite(LYNX_C_LED, state == AGENT_CONNECTED ? HIGH : LOW);
        digitalWrite(LYNX_D_LED, state == AGENT_DISCONNECTED ? HIGH : LOW);
        delay(100);
    });

}

void loop() {
    vTaskDelete(NULL);
}
