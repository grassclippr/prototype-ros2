#ifndef LEDS_TASK_H
#define LEDS_TASK_H

#include <BlinkControl.h>

#include "../../hardware.h"
#include "driver/ledc.h"
#include "tasks/uros/client.h"

class LedControl {
   public:
    void setup();
    static void task(void *arg);

    BlinkControl status_led = BlinkControl(STATUS_LED, LEDC_CHANNEL_0, 4000, LEDC_TIMER_8_BIT);

   private:
};

#endif  // LEDS_TASK_H
