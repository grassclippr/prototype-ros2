#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include "driver/ledc.h"

class MotorControl {
   public:
    struct Command {
        float linear_x = 0.0f;
        float angular_z = 0.0f;
        uint32_t seq = 0;
        uint32_t timeout_ms = 0;
        uint32_t last_update_ms = 0;
        bool active = false;
    };

    void setup();
    static void task(void *arg);
    void setCommand(float linear_x, float angular_z, uint32_t seq, uint32_t timeout_ms);
    void stop();
    bool expireIfTimedOut(uint32_t now_ms, uint32_t *expired_seq);
    Command getCommand() const;

   private:
    volatile float linear_x_ = 0.0f;
    volatile float angular_z_ = 0.0f;
    volatile uint32_t seq_ = 0;
    volatile uint32_t timeout_ms_ = 0;
    volatile uint32_t last_update_ms_ = 0;
    volatile bool active_ = false;
};

#endif  // MOTOR_CONTROL_TASK_H
