#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <nmea_msgs/msg/sentence.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include "tasks/leds/task.h"
#include "tasks/motors/task.h"
#include "tasks/uros/client.h"

class Rover {
   public:
    Rover();
    void startPairing();
    void stopPairing();

   private:
    // Tasks
    LedControl leds;
    UrosClient uros_client;
    MotorControl motors;
    static void pairingTask(void *arg);
    static void gnssReceiveTask(void *arg);

    void sendNmeaCommand(const String &cmd);

    // ROS communication
    rcl_timer_t timer;
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;

    rcl_publisher_t nmea_publisher;
    nmea_msgs__msg__Sentence nmea_msg;

    rcl_subscription_t cmd_vel_sub;
    geometry_msgs__msg__Twist cmd_vel_msg;

    // ESP-NOW communication
    void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);

    TaskHandle_t pairingTaskHandle = nullptr;
    bool paired = false;
};
