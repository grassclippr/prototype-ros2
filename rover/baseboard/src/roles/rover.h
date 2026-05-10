#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <freertos/queue.h>
#include <nmea_msgs/msg/sentence.h>
#include <rover_baseboard_msgs/msg/encoder_state.h>
#include <rover_baseboard_msgs/msg/motor_command.h>
#include <rover_baseboard_msgs/msg/safety_state.h>
#include <std_msgs/msg/int32.h>

#include "tasks/leds/task.h"
#include "tasks/motors/task.h"
#include "tasks/uros/client.h"

class Rover {
   public:
    Rover();
    void startPairing();
    void stopPairing();
    bool isRosConnected() const;
    void requestRosReconnect(const char *reason);

   private:
    struct PendingNmeaSentence {
        size_t len;
        char data[96];
    };

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
    rcl_timer_t nmea_publish_timer = rcl_get_zero_initialized_timer();
    QueueHandle_t nmea_publish_queue = nullptr;

    rcl_subscription_t motor_command_sub = rcl_get_zero_initialized_subscription();
    rover_baseboard_msgs__msg__MotorCommand motor_command_msg;

    rcl_timer_t encoder_state_timer = rcl_get_zero_initialized_timer();
    rcl_publisher_t encoder_state_publisher = rcl_get_zero_initialized_publisher();
    rover_baseboard_msgs__msg__EncoderState encoder_state_msg;
    bool encoder_state_publisher_initialized = false;
    bool encoder_state_timer_initialized = false;

    rcl_timer_t safety_state_timer = rcl_get_zero_initialized_timer();
    rcl_publisher_t safety_state_publisher = rcl_get_zero_initialized_publisher();
    rover_baseboard_msgs__msg__SafetyState safety_state_msg;
    bool safety_state_publisher_initialized = false;
    bool safety_state_timer_initialized = false;

    // ESP-NOW communication
    void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);

    TaskHandle_t pairingTaskHandle = nullptr;
    TaskHandle_t heartbeatTaskHandle = nullptr;
    TaskHandle_t commandWatchdogTaskHandle = nullptr;
    bool paired = false;
    bool baseboard_publisher_initialized = false;
    bool nmea_publisher_initialized = false;
    bool nmea_publish_timer_initialized = false;
    bool motor_command_msg_initialized = false;
    bool motor_command_sub_initialized = false;
    bool encoder_state_msg_initialized = false;
    bool safety_state_msg_initialized = false;
    bool timer_initialized = false;
};
