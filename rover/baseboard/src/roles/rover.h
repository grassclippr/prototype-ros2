#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <geometry_msgs/msg/twist.h>
#include <nmea_msgs/msg/sentence.h>
#include <std_msgs/msg/float32_multi_array.h>
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

    rcl_subscription_t wheel_cmd_sub = rcl_get_zero_initialized_subscription();
    std_msgs__msg__Float32MultiArray wheel_cmd_msg;

    // Wheel velocity publisher (measured from encoders)
    rcl_timer_t odom_vel_timer = rcl_get_zero_initialized_timer();
    rcl_publisher_t odom_vel_publisher = rcl_get_zero_initialized_publisher();
    geometry_msgs__msg__Twist odom_vel_msg;
    bool odom_vel_publisher_initialized = false;
    bool odom_vel_timer_initialized = false;

    // Wheel velocity publisher (measured from encoders)
    rcl_timer_t odom_vel_timer = rcl_get_zero_initialized_timer();
    rcl_publisher_t odom_vel_publisher = rcl_get_zero_initialized_publisher();
    geometry_msgs__msg__Twist odom_vel_msg;
    bool odom_vel_publisher_initialized = false;
    bool odom_vel_timer_initialized = false;

    // ESP-NOW communication
    void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);

    TaskHandle_t pairingTaskHandle = nullptr;
    TaskHandle_t heartbeatTaskHandle = nullptr;
    TaskHandle_t commandWatchdogTaskHandle = nullptr;
    bool paired = false;
    bool baseboard_publisher_initialized = false;
    bool nmea_publisher_initialized = false;
    bool wheel_cmd_msg_initialized = false;
    bool wheel_cmd_sub_initialized = false;
    bool timer_initialized = false;
};
