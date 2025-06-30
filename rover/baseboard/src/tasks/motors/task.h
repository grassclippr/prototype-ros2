#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include <geometry_msgs/msg/twist.h>

#include "driver/ledc.h"

class MotorControl {
   public:
    void setup();
    static void task(void *arg);

    void setVelocities(const geometry_msgs__msg__Twist *msg) {
        // Implement logic to set motor velocities based on the Twist message
        // For example, you might map linear.x to left/right motor speeds
        // and angular.z to turning speed.
        float left_speed = msg->linear.x - msg->angular.z;
        float right_speed = msg->linear.x + msg->angular.z;
    };

   private:
};

#endif  // MOTOR_CONTROL_TASK_H