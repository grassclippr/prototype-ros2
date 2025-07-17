#ifndef LEDS_TASK_H
#define LEDS_TASK_H

//#include <BlinkControl.h>
#include <OneButton.h>

#include "../../hardware.h"
//#include "driver/ledc.h"
#include "tasks/uros/client.h"

class LedControl {
   public:
    void setup();
    static void task(void *arg);

    //BlinkControl status_led = BlinkControl(STATUS_LED, LEDC_CHANNEL_7, 4000, LEDC_TIMER_8_BIT);
    OneButton bootButton;

   private:
};

#endif  // LEDS_TASK_H
