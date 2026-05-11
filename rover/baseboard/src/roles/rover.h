#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include <nmea_msgs/msg/sentence.h>
#include <rover_baseboard_msgs/msg/encoder_state.h>
#include <rover_baseboard_msgs/msg/motor_command.h>
#include <rover_baseboard_msgs/msg/safety_state.h>
#include <std_msgs/msg/int32.h>

#define ROVER_GNSS_TRANSPORT_RAW_NMEA 0
#define ROVER_GNSS_TRANSPORT_STRUCTURED_FIX 1

#ifndef ROVER_GNSS_TRANSPORT_MODE
#define ROVER_GNSS_TRANSPORT_MODE ROVER_GNSS_TRANSPORT_STRUCTURED_FIX
#endif

#if ROVER_GNSS_TRANSPORT_MODE == ROVER_GNSS_TRANSPORT_STRUCTURED_FIX
#include <rover_baseboard_msgs/msg/gnss_fix.h>
#endif

#include "tasks/leds/task.h"
#include "tasks/motors/task.h"
#include "tasks/uros/client.h"

class Rover {
   public:
#if ROVER_GNSS_TRANSPORT_MODE == ROVER_GNSS_TRANSPORT_STRUCTURED_FIX
    struct UtcDate {
        bool valid = false;
        uint16_t year = 0;
        uint8_t month = 0;
        uint8_t day = 0;
    };

    struct LatestGnssFix {
        bool pending = false;
        bool utc_valid = false;
        uint16_t utc_year = 0;
        uint8_t utc_month = 0;
        uint8_t utc_day = 0;
        uint8_t utc_hour = 0;
        uint8_t utc_minute = 0;
        float utc_second = 0.0f;
        double latitude_deg = 0.0;
        double longitude_deg = 0.0;
        double altitude_m = 0.0;
        float hdop = 0.0f;
        uint8_t satellites = 0;
        uint8_t fix_quality = 0;
    };
#endif

    Rover();
    void startPairing();
    void stopPairing();
    bool isRosConnected() const;
    void requestRosReconnect(const char *reason);

   private:
    struct LatestNmeaSentence {
        bool pending = false;
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
    portMUX_TYPE nmea_publish_mux = portMUX_INITIALIZER_UNLOCKED;
    LatestNmeaSentence latest_gga_sentence{};
    LatestNmeaSentence latest_rmc_sentence{};
    bool publish_gga_next = true;

#if ROVER_GNSS_TRANSPORT_MODE == ROVER_GNSS_TRANSPORT_STRUCTURED_FIX
    rcl_publisher_t gnss_fix_publisher = rcl_get_zero_initialized_publisher();
    rover_baseboard_msgs__msg__GnssFix gnss_fix_msg;
    rcl_timer_t gnss_fix_publish_timer = rcl_get_zero_initialized_timer();
    LatestGnssFix latest_gnss_fix{};
    UtcDate latest_rmc_date{};
#endif

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
#if ROVER_GNSS_TRANSPORT_MODE == ROVER_GNSS_TRANSPORT_STRUCTURED_FIX
    bool gnss_fix_publisher_initialized = false;
    bool gnss_fix_msg_initialized = false;
    bool gnss_fix_publish_timer_initialized = false;
#endif
    bool motor_command_msg_initialized = false;
    bool motor_command_sub_initialized = false;
    bool encoder_state_msg_initialized = false;
    bool safety_state_msg_initialized = false;
    bool timer_initialized = false;
};
