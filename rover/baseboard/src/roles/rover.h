#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <nmea_msgs/msg/sentence.h>
#include <rover_msgs/msg/drive_command.h>
#include <rover_msgs/msg/drive_command_ack.h>
#include <rosidl_runtime_c/string_functions.h>
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
    void publishDriveAck(uint32_t seq, uint8_t status, const char *detail);
    void stopMotion();
    bool validateDriveCommand(const rover_msgs__msg__DriveCommand *msg, const char **detail) const;

    // ROS communication
    rcl_timer_t timer;
    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;

    rcl_publisher_t nmea_publisher;
    nmea_msgs__msg__Sentence nmea_msg;

    rcl_publisher_t drive_ack_publisher;
    rover_msgs__msg__DriveCommandAck drive_ack_msg;

    rcl_subscription_t drive_command_sub;
    rover_msgs__msg__DriveCommand drive_command_msg;

    // ESP-NOW communication
    void onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len);

    TaskHandle_t pairingTaskHandle = nullptr;
    TaskHandle_t heartbeatTaskHandle = nullptr;
    TaskHandle_t commandWatchdogTaskHandle = nullptr;
    bool paired = false;
    bool drive_command_msg_initialized = false;
    bool drive_ack_msg_initialized = false;
    bool baseboard_publisher_initialized = false;
    bool nmea_publisher_initialized = false;
    bool drive_ack_publisher_initialized = false;
    bool drive_command_sub_initialized = false;
    bool timer_initialized = false;
};
