#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include "driver/ledc.h"

class MotorControl {
   public:
    void setup();
    static void task(void *arg);

   private:
};

#endif  // MOTOR_CONTROL_TASK_H