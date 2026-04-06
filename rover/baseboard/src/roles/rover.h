#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <geometry_msgs/msg/twist.h>
#include <nmea_msgs/msg/sentence.h>
#include <std_msgs/msg/int32.h>

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
    static void heartbeatTask(void *arg);
    static void commandWatchdogTask(void *arg);

    void sendNmeaCommand(const String &cmd);
    void stopMotion();

    // ROS communication
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    std_msgs__msg__Int32 msg;

    rcl_publisher_t nmea_publisher = rcl_get_zero_initialized_publisher();
    nmea_msgs__msg__Sentence nmea_msg;

    rcl_subscription_t cmd_vel_sub = rcl_get_zero_initialized_subscription();
    geometry_msgs__msg__Twist cmd_vel_msg;

    // ESP-NOW communication
    void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);

    TaskHandle_t pairingTaskHandle = nullptr;
    TaskHandle_t heartbeatTaskHandle = nullptr;
    TaskHandle_t commandWatchdogTaskHandle = nullptr;
    bool paired = false;
    bool baseboard_publisher_initialized = false;
    bool nmea_publisher_initialized = false;
    bool cmd_vel_sub_initialized = false;
    bool timer_initialized = false;
};
