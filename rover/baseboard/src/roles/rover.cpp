#include "./rover.h"

#include "tasks/uros/error.h"

#include <cmath>

#include <rmw/qos_profiles.h>
#include "./espnow.h"

#ifndef SERIAL_MUX_HEARTBEAT
#define SERIAL_MUX_HEARTBEAT 1
#endif

#ifndef SERIAL_MUX_DISABLE_ROS
#define SERIAL_MUX_DISABLE_ROS 0
#endif

#ifndef ROVER_ENABLE_NMEA_PUBLISHER
#define ROVER_ENABLE_NMEA_PUBLISHER 1
#endif

constexpr uint32_t WHEEL_CMD_TIMEOUT_MS = 500;
constexpr uint32_t GNSS_READ_TIMEOUT_MS = 200;
constexpr size_t MAX_NMEA_SENTENCE_LEN = 82;
constexpr size_t GNSS_UART_RX_BUFFER_SIZE = 4096;
constexpr uint32_t NMEA_PUBLISH_PERIOD_MS = 20;

static Rover *selfRover = nullptr;

namespace {
constexpr uint32_t BASEBOARD_HEARTBEAT_MS = 5000;
constexpr uint8_t BASEBOARD_FAILURES_BEFORE_RECONNECT = 0;
constexpr uint8_t WHEEL_VELOCITY_FAILURES_BEFORE_RECONNECT = 20;

bool waitForSerialBytes(HardwareSerial &serial, size_t count, uint32_t timeout_ms) {
    const unsigned long deadline = millis() + timeout_ms;
    while (serial.available() < static_cast<int>(count)) {
        if (millis() >= deadline) {
            return false;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    return true;
}

void discardUntilNewline(HardwareSerial &serial, uint32_t timeout_ms) {
    unsigned long deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        while (serial.available()) {
            int raw = serial.read();
            if (raw < 0) {
                continue;
            }
            if (static_cast<char>(raw) == '\n') {
                return;
            }
            deadline = millis() + timeout_ms;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

bool readNmeaLine(HardwareSerial &serial, String &line, uint32_t timeout_ms) {
    line = "";
    unsigned long deadline = millis() + timeout_ms;
    while (true) {
        while (!serial.available()) {
            if (millis() >= deadline) {
                if (line.length() > 0) {
                    discardUntilNewline(serial, timeout_ms);
                }
                return false;
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        int raw = serial.read();
        if (raw < 0) {
            continue;
        }

        char ch = static_cast<char>(raw);
        if (ch == '\n') {
            return line.length() > 0;
        }
        if (ch == '\r') {
            continue;
        }

        line += ch;
        deadline = millis() + timeout_ms;
        if (line.length() > MAX_NMEA_SENTENCE_LEN) {
            discardUntilNewline(serial, timeout_ms);
            return false;
        }
    }
}

bool shouldForwardNmeaSentence(const String &line) {
    // Keep the initial GNSS bridge focused on the minimum sentences needed for
    // navigation bring-up. Extra chatter such as GSV/GSA can overload the shared
    // micro-ROS transport without improving the core fix path.
    return line.startsWith("$GNGGA") ||
           line.startsWith("$GPGGA") ||
           line.startsWith("$GNRMC") ||
           line.startsWith("$GPRMC");
}

enum class NmeaSentenceType {
    None,
    Gga,
    Rmc,
};

NmeaSentenceType classifyNmeaSentence(const String &line) {
    if (line.startsWith("$GNGGA") || line.startsWith("$GPGGA")) {
        return NmeaSentenceType::Gga;
    }
    if (line.startsWith("$GNRMC") || line.startsWith("$GPRMC")) {
        return NmeaSentenceType::Rmc;
    }
    return NmeaSentenceType::None;
}

bool publishWithReconnect(
    rcl_publisher_t *publisher,
    const void *ros_message,
    const char *publisher_name,
    uint8_t failures_before_reconnect,
    uint8_t &consecutive_failures,
    uint32_t &last_error_log_ms)
{
    if (!selfRover->isRosConnected()) {
        consecutive_failures = 0;
        return false;
    }

    const rcl_ret_t rc = rcl_publish(publisher, ros_message, NULL);
    if (rc == RCL_RET_OK) {
        if (consecutive_failures > 0) {
            printf("%s recovered after %u publish failure(s)\n",
                   publisher_name,
                   static_cast<unsigned>(consecutive_failures));
        }
        consecutive_failures = 0;
        return true;
    }

    consecutive_failures++;
    const uint32_t now_ms = millis();
    if (now_ms - last_error_log_ms >= 1000) {
        printf("Failed to publish %s (rc=%d, failures=%u)\n",
                   publisher_name,
                   static_cast<int>(rc),
                   static_cast<unsigned>(consecutive_failures));
        last_error_log_ms = now_ms;
    }
    if (failures_before_reconnect > 0 && consecutive_failures >= failures_before_reconnect) {
        printf("Requesting uros reconnect after %u consecutive %s failure(s)\n",
               static_cast<unsigned>(consecutive_failures),
               publisher_name);
        selfRover->requestRosReconnect(publisher_name);
    }
    return false;
}

}  // namespace

Rover::Rover() {
    selfRover = this;

    USBSerial.begin(921600);
    leds.setup();
    motors.setup();
#if !SERIAL_MUX_DISABLE_ROS
    uros_client.setup(USBSerial);
#endif

    leds.bootButton.attachClick([]() {
        if (selfRover->pairingTaskHandle == nullptr) {
            printf("Starting pairing mode\n");
            selfRover->startPairing();
        } else {
            printf("Stopping pairing mode\n");
            selfRover->stopPairing();
        }
    });

    uros_client.subscribeToStateChange([&](ClientState state) {
        if (state == AGENT_CONNECTED) {
            digitalWrite(STATUS_LED, HIGH);
            //leds.status_led.on();
        } else {
            digitalWrite(STATUS_LED, LOW);
            //leds.status_led.blink1();
            selfRover->stopMotion();
        }

        switch (state) {
            case AGENT_CONNECTED:
                digitalWrite(LYNX_C_LED, HIGH);
                digitalWrite(LYNX_D_LED, HIGH);
                break;
            case AGENT_DISCONNECTED:
                digitalWrite(LYNX_C_LED, LOW);
                digitalWrite(LYNX_D_LED, LOW);
                break;
            case WAITING_AGENT:
                digitalWrite(LYNX_C_LED, LOW);
                digitalWrite(LYNX_D_LED, HIGH);
                break;
            case CONNECTING:
                digitalWrite(LYNX_C_LED, HIGH);
                digitalWrite(LYNX_D_LED, LOW);
            default:
                break;
                delay(10);
        }
    });
    uros_client.onCreateEntities([&](rcl_node_t *node, rclc_support_t *support) {
        msg.data = 0;
        publisher = rcl_get_zero_initialized_publisher();
        nmea_publisher = rcl_get_zero_initialized_publisher();
        motor_command_sub = rcl_get_zero_initialized_subscription();
        // Use a fixed-size message type here so micro-ROS can deserialize wheel
        // commands without dynamic allocation on the MCU.
        if (!rover_baseboard_msgs__msg__MotorCommand__init(&motor_command_msg)) {
            printf("Failed to initialize motor command message buffer\n");
            return false;
        }
        motor_command_msg_initialized = true;

        rcl_ret_t rc = RCL_RET_ERROR;
        rmw_qos_profile_t baseboard_qos = rmw_qos_profile_sensor_data;
        baseboard_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        baseboard_qos.depth = 1;
        rc = rclc_publisher_init(
            &publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "baseboard",
            &baseboard_qos);
        if (rc != RCL_RET_OK) {
            log_rcl_error("baseboard publisher init", rc);
            return false;
        }
        baseboard_publisher_initialized = true;

#if ROVER_ENABLE_NMEA_PUBLISHER
        rc = rclc_publisher_init_default(
            &nmea_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nmea_msgs, msg, Sentence),
            "/baseboard/nmea_sentence_raw");
        if (rc != RCL_RET_OK) {
            log_rcl_error("nmea publisher init", rc);
            return false;
        }
        nmea_publisher_initialized = true;

        nmea_publish_timer = rcl_get_zero_initialized_timer();
        rc = rclc_timer_init_default2(
            &nmea_publish_timer,
            support,
            RCL_MS_TO_NS(NMEA_PUBLISH_PERIOD_MS),
            [](rcl_timer_t *timer, int64_t last_call_time) {
                (void)timer;
                (void)last_call_time;
                LatestNmeaSentence pending{};
                taskENTER_CRITICAL(&selfRover->nmea_publish_mux);
                if (selfRover->latest_gga_sentence.pending) {
                    pending = selfRover->latest_gga_sentence;
                    selfRover->latest_gga_sentence.pending = false;
                } else if (selfRover->latest_rmc_sentence.pending) {
                    pending = selfRover->latest_rmc_sentence;
                    selfRover->latest_rmc_sentence.pending = false;
                }
                taskEXIT_CRITICAL(&selfRover->nmea_publish_mux);

                if (!pending.pending) {
                    return;
                }

                memset(selfRover->nmea_msg.sentence.data, 0, selfRover->nmea_msg.sentence.capacity);
                memcpy(selfRover->nmea_msg.sentence.data, pending.data, pending.len);
                selfRover->nmea_msg.sentence.data[pending.len] = '\0';
                selfRover->nmea_msg.sentence.size = pending.len;

                const uint32_t now_ms = millis();
                selfRover->nmea_msg.header.stamp.sec = now_ms / 1000;
                selfRover->nmea_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000UL;

                static uint8_t consecutive_failures = 0;
                static uint32_t last_error_log_ms = 0;
                (void)publishWithReconnect(
                    &selfRover->nmea_publisher,
                    &selfRover->nmea_msg,
                    "nmea publisher",
                    WHEEL_VELOCITY_FAILURES_BEFORE_RECONNECT,
                    consecutive_failures,
                    last_error_log_ms);
            },
            true);
        if (rc != RCL_RET_OK) {
            log_rcl_error("nmea publish timer init", rc);
            return false;
        }
        nmea_publish_timer_initialized = true;
#endif

        // Subscription creation can fail if the XRCE session isn't
        // fully ready yet.  Retry with a short delay to let the
        // session settle.
        constexpr int kMaxSubRetries = 3;
        constexpr int kSubRetryDelayMs = 500;
        rmw_qos_profile_t motor_command_qos = rmw_qos_profile_default;
        motor_command_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        motor_command_qos.depth = 1;
        motor_command_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        motor_command_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        rc = RCL_RET_ERROR;
        for (int attempt = 1; attempt <= kMaxSubRetries; ++attempt) {
            if (attempt > 1) {
                delay(kSubRetryDelayMs);
                rcl_reset_error();
                motor_command_sub = rcl_get_zero_initialized_subscription();
            }
            rc = rclc_subscription_init(
                &motor_command_sub,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(rover_baseboard_msgs, msg, MotorCommand),
                "/baseboard/motor_command",
                &motor_command_qos);
            if (rc == RCL_RET_OK) {
                break;
            }
            log_rcl_error("motor_command subscription init", rc);
        }
        if (rc != RCL_RET_OK) {
            return false;
        }
        motor_command_sub_initialized = true;

        const uint32_t timer_timeout = BASEBOARD_HEARTBEAT_MS;
        rc = rclc_timer_init_default2(
            &timer,
            support,
            RCL_MS_TO_NS(timer_timeout),
            [](rcl_timer_t *timer, int64_t last_call_time) {
                (void)timer;
                (void)last_call_time;
                static uint8_t consecutive_failures = 0;
                static uint32_t last_error_log_ms = 0;
                if (publishWithReconnect(
                        &selfRover->publisher,
                        &selfRover->msg,
                        "baseboard publisher",
                        BASEBOARD_FAILURES_BEFORE_RECONNECT,
                        consecutive_failures,
                        last_error_log_ms)) {
                    selfRover->msg.data++;
                }
            },
            true);
        if (rc != RCL_RET_OK) {
            log_rcl_error("baseboard timer init", rc);
            return false;
        }
        timer_initialized = true;

        // Encoder state publisher
        encoder_state_publisher = rcl_get_zero_initialized_publisher();
        rover_baseboard_msgs__msg__EncoderState__init(&encoder_state_msg);
        encoder_state_msg_initialized = true;
        rmw_qos_profile_t wheel_feedback_qos = rmw_qos_profile_sensor_data;
        wheel_feedback_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        wheel_feedback_qos.depth = 1;
        rc = rclc_publisher_init(
            &encoder_state_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(rover_baseboard_msgs, msg, EncoderState),
            "/baseboard/encoder_state",
            &wheel_feedback_qos);
        if (rc != RCL_RET_OK) {
            log_rcl_error("encoder_state publisher init", rc);
            return false;
        }
        encoder_state_publisher_initialized = true;

        safety_state_publisher = rcl_get_zero_initialized_publisher();
        rover_baseboard_msgs__msg__SafetyState__init(&safety_state_msg);
        safety_state_msg_initialized = true;
        rmw_qos_profile_t safety_state_qos = rmw_qos_profile_sensor_data;
        safety_state_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        safety_state_qos.depth = 1;
        rc = rclc_publisher_init(
            &safety_state_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(rover_baseboard_msgs, msg, SafetyState),
            "/baseboard/safety_state",
            &safety_state_qos);
        if (rc != RCL_RET_OK) {
            log_rcl_error("safety_state publisher init", rc);
            return false;
        }
        safety_state_publisher_initialized = true;

        // 20 Hz timer to publish encoder feedback
        encoder_state_timer = rcl_get_zero_initialized_timer();
        rc = rclc_timer_init_default2(
            &encoder_state_timer,
            support,
            RCL_MS_TO_NS(50),  // 50ms = 20Hz
            [](rcl_timer_t *timer, int64_t last_call_time) {
                (void)timer;
                (void)last_call_time;
                auto left  = selfRover->motors.getLeftWheel();
                auto right = selfRover->motors.getRightWheel();

                selfRover->encoder_state_msg.left_ticks = left.ticks;
                selfRover->encoder_state_msg.right_ticks = right.ticks;
                selfRover->encoder_state_msg.left_velocity_ticks_per_sec = left.velocity_ticks_per_sec;
                selfRover->encoder_state_msg.right_velocity_ticks_per_sec = right.velocity_ticks_per_sec;

                static uint8_t consecutive_failures = 0;
                static uint32_t last_error_log_ms = 0;
                (void)publishWithReconnect(
                    &selfRover->encoder_state_publisher,
                    &selfRover->encoder_state_msg,
                    "encoder state publisher",
                    WHEEL_VELOCITY_FAILURES_BEFORE_RECONNECT,
                    consecutive_failures,
                    last_error_log_ms);
            },
            true);
        if (rc != RCL_RET_OK) {
            log_rcl_error("encoder_state timer init", rc);
            return false;
        }
        encoder_state_timer_initialized = true;

        safety_state_timer = rcl_get_zero_initialized_timer();
        rc = rclc_timer_init_default2(
            &safety_state_timer,
            support,
            RCL_MS_TO_NS(100),  // 100ms = 10Hz
            [](rcl_timer_t *timer, int64_t last_call_time) {
                (void)timer;
                (void)last_call_time;
                auto state = selfRover->motors.getSafetyState();
                selfRover->safety_state_msg.interlock_triggered = state.interlock_triggered;
                selfRover->safety_state_msg.lift_triggered = state.lift_triggered;
                selfRover->safety_state_msg.bumper_one_wire_triggered = state.bumper_one_wire_triggered;
                selfRover->safety_state_msg.bumper_uart_fault_triggered = state.bumper_uart_fault_triggered;

                static uint8_t consecutive_failures = 0;
                static uint32_t last_error_log_ms = 0;
                (void)publishWithReconnect(
                    &selfRover->safety_state_publisher,
                    &selfRover->safety_state_msg,
                    "safety state publisher",
                    WHEEL_VELOCITY_FAILURES_BEFORE_RECONNECT,
                    consecutive_failures,
                    last_error_log_ms);
            },
            true);
        if (rc != RCL_RET_OK) {
            log_rcl_error("safety_state timer init", rc);
            return false;
        }
        safety_state_timer_initialized = true;

        return true;
    });
    uros_client.onExecutorInit(5, [&](rclc_executor_t *executor) {
        RCCHECK(rclc_executor_add_timer(executor, &timer));
        RCCHECK(rclc_executor_add_timer(executor, &encoder_state_timer));
        RCCHECK(rclc_executor_add_timer(executor, &safety_state_timer));
#if ROVER_ENABLE_NMEA_PUBLISHER
        RCCHECK(rclc_executor_add_timer(executor, &nmea_publish_timer));
#endif
        RCCHECK(rclc_executor_add_subscription(
            executor,
            &motor_command_sub,
            &motor_command_msg,
            [](const void *msgin) -> void {
                auto *command = static_cast<const rover_baseboard_msgs__msg__MotorCommand *>(msgin);
                const float left = command->left_speed_percent;
                const float right = command->right_speed_percent;
                if (!std::isfinite(left) || !std::isfinite(right)) {
                    printf("Rejected motor_command: non-finite speed percent\n");
                    return;
                }

                const uint32_t timeout_ms =
                    command->timeout_ms > 0 ? command->timeout_ms : WHEEL_CMD_TIMEOUT_MS;
                selfRover->motors.setWheelCommand(left, right, 0, timeout_ms);
            },
            ON_NEW_DATA));
        return true;
    });

    // Cleanup when connection is lost
    uros_client.onDestroyEntities([&](rcl_node_t *node, rclc_support_t *support) {
        (void)support;
        if (timer_initialized) {
            RCSOFTCHECK(rcl_timer_fini(&timer));
            timer_initialized = false;
        }
        if (baseboard_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&publisher, node));
            baseboard_publisher_initialized = false;
        }
#if ROVER_ENABLE_NMEA_PUBLISHER
        if (nmea_publish_timer_initialized) {
            RCSOFTCHECK(rcl_timer_fini(&nmea_publish_timer));
            nmea_publish_timer_initialized = false;
        }
        if (nmea_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&nmea_publisher, node));
            nmea_publisher_initialized = false;
        }
#endif
        if (motor_command_sub_initialized) {
            RCSOFTCHECK(rcl_subscription_fini(&motor_command_sub, node));
            motor_command_sub_initialized = false;
        }
        if (motor_command_msg_initialized) {
            rover_baseboard_msgs__msg__MotorCommand__fini(&motor_command_msg);
            motor_command_msg_initialized = false;
        }
        if (encoder_state_timer_initialized) {
            RCSOFTCHECK(rcl_timer_fini(&encoder_state_timer));
            encoder_state_timer_initialized = false;
        }
        if (encoder_state_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&encoder_state_publisher, node));
            encoder_state_publisher_initialized = false;
        }
        if (encoder_state_msg_initialized) {
            rover_baseboard_msgs__msg__EncoderState__fini(&encoder_state_msg);
            encoder_state_msg_initialized = false;
        }
        if (safety_state_timer_initialized) {
            RCSOFTCHECK(rcl_timer_fini(&safety_state_timer));
            safety_state_timer_initialized = false;
        }
        if (safety_state_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&safety_state_publisher, node));
            safety_state_publisher_initialized = false;
        }
        if (safety_state_msg_initialized) {
            rover_baseboard_msgs__msg__SafetyState__fini(&safety_state_msg);
            safety_state_msg_initialized = false;
        }
    });

    // Initialize ESP-NOW
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
            selfRover->onEspNowRecv(mac, data, len);
        });
        // esp_now_register_send_cb(onEspNowSent);

        // Make sure to load the stored MAC from NVS
        loadPeerFromNVS();
    }

    // Start GNSS receive task
    xTaskCreate(
        gnssReceiveTask,
        "gnssReceiveTask",
        4000,
        this,
        1,
        nullptr);

#if SERIAL_MUX_HEARTBEAT
    xTaskCreate(
        heartbeatTask,
        "heartbeatTask",
        4096,
        this,
        1,
        &heartbeatTaskHandle);
#endif

    xTaskCreate(
        commandWatchdogTask,
        "commandWatchdogTask",
        4096,
        this,
        1,
        &commandWatchdogTaskHandle);
};

bool Rover::isRosConnected() const {
    return uros_client.isConnected();
}

void Rover::requestRosReconnect(const char *reason) {
    uros_client.requestReconnect(reason);
}

void Rover::gnssReceiveTask(void *arg) {
    Rover *self = (Rover *)arg;

    // GNSS emits bursts at 460800 baud; a larger RX buffer helps avoid
    // dropped bytes while this task is publishing prior sentences.
    Serial2.setRxBufferSize(GNSS_UART_RX_BUFFER_SIZE);
    Serial2.begin(460800, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    self->sendNmeaCommand("PQTMCFGRCVRMODE,W,1"); // Set receiver to rover mode (accept RTCM corrections)
    delay(100);
    self->sendNmeaCommand("PQTMCFGMSGRATE,W,GGA,1,1"); // Ensure fix/altitude updates are available.
    delay(30);
    self->sendNmeaCommand("PQTMCFGMSGRATE,W,RMC,1,1"); // Ensure position/time/course updates are available.
    delay(30);
    self->sendNmeaCommand("PAIR062,0,1");
    delay(30);
    self->sendNmeaCommand("QTMSAVEPAR"); // Save settings to non-volatile memory, so they persist after reboot
    delay(100);

    if (!nmea_msgs__msg__Sentence__init(&self->nmea_msg)) {
        printf("Failed to initialize NMEA sentence message\n");
        vTaskDelete(nullptr);
        return;
    }

    free(self->nmea_msg.sentence.data);
    self->nmea_msg.sentence.data = (char *)malloc(82 + 1 * sizeof(char));
    if (self->nmea_msg.sentence.data == nullptr) {
        printf("Failed to allocate NMEA sentence buffer\n");
        vTaskDelete(nullptr);
        return;
    }
    self->nmea_msg.sentence.size = 0;
    self->nmea_msg.sentence.capacity = 82 + 1;  // 82 is the maximum length of NMEA sentences, +1 for null terminator

    uint32_t last_status_print_ms = 0;

    while (true) {
        if (Serial2.available() < 1) {
            vTaskDelay(1);
            continue;
        }
        int first = Serial2.peek();

        switch (first) {
            case 0xD3: {
                // RTCM message
                Serial2.read();  // consume first byte
                int l1 = Serial2.read();
                int l2 = Serial2.read();
                if (l1 < 0 || l2 < 0) {
                    printf("Error reading RTCM length bytes\n");
                    break;
                }
                int length = l1 * 256 + l2;

                if (length <= 0 || length > 1023) {
                    printf("Invalid RTCM length: %d\n", length);
                    // Optionally, flush serial buffer here
                    break;
                }

                // Read RTCM payload
                std::vector<uint8_t> buf(length);
                int read_bytes = Serial2.readBytes(buf.data(), length);
                if (read_bytes != length) {
                    printf("Error reading RTCM payload (%d/%d bytes)\n", read_bytes, length);
                    break;
                }

                // Read checksum (3 bytes)
                uint8_t checksum[3];
                int cs_read = Serial2.readBytes(checksum, 3);
                if (cs_read != 3) {
                    printf("Error reading RTCM checksum\n");
                    break;
                }

                // Ignore the message, sice we don't use RTCM messages in the rover
                break;
            }
            case '$': {
                // NMEA or proprietary message
                String line;
                if (!readNmeaLine(Serial2, line, GNSS_READ_TIMEOUT_MS)) {
                    printf("Error reading NMEA line\n");
                    break;
                }

#if ROVER_ENABLE_NMEA_PUBLISHER
                const NmeaSentenceType sentence_type = classifyNmeaSentence(line);
                if (line.length() <= MAX_NMEA_SENTENCE_LEN &&
                    sentence_type != NmeaSentenceType::None &&
                    shouldForwardNmeaSentence(line) &&
                    self->uros_client.isConnected() &&
                    self->nmea_publisher_initialized) {
                    taskENTER_CRITICAL(&self->nmea_publish_mux);
                    LatestNmeaSentence *slot =
                        sentence_type == NmeaSentenceType::Gga
                            ? &self->latest_gga_sentence
                            : &self->latest_rmc_sentence;
                    slot->pending = true;
                    slot->len = line.length();
                    memcpy(slot->data, line.c_str(), slot->len);
                    slot->data[slot->len] = '\0';
                    taskEXIT_CRITICAL(&self->nmea_publish_mux);
                }
#endif
                break;
            }
            default:
                Serial2.read();  // consume unknown byte
                // printf("Unknown first byte: 0x%02X\n", first);
                break;
        }
        yield();
    }
}

void Rover::commandWatchdogTask(void *arg) {
    Rover *self = static_cast<Rover *>(arg);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        xTaskDelayUntil(&xLastWakeTime, 20 / portTICK_RATE_MS);

        self->motors.expireIfTimedOut(millis(), nullptr);
    }
}

void Rover::heartbeatTask(void *arg) {
    Rover *self = (Rover *)arg;
    (void)self;

    uint32_t counter = 0;
    while (true) {
        printf("h%lu\n", static_cast<unsigned long>(counter));
        counter++;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void Rover::stopMotion() {
    motors.stop();
}

void Rover::sendNmeaCommand(const String &cmd) {
    // Calculate checksum (XOR of all bytes)
    uint8_t checksum = 0;
    for (size_t i = 0; i < cmd.length(); ++i) {
        checksum ^= cmd[i];
    }
    // Format and send: $<cmd>*<checksum>\r\n
    char buf[128];
    snprintf(buf, sizeof(buf), "$%s*%02X\r\n", cmd.c_str(), checksum);
    Serial2.print(buf);
}

void Rover::onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len) {
    if (len < 1) {
        return;
    }

    switch (data[0]) {
        case MSG_TYPE_PAIR_REQ: {
            if (pairingTaskHandle == nullptr) {
                printf("Pairing request received from %02x:%02x:%02x:%02x:%02x:%02x, but not in pairing mode\n",
                           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                return;
            }

            addPeer(mac_addr);

            uint8_t ack[] = {MSG_TYPE_PAIR_ACK};  // Answer pair request with ACK
            esp_now_send(mac_addr, ack, sizeof(ack));
            paired = true;
            stopPairing();
            break;
        }
        case MSG_TYPE_RTCM:
            //printf("Base RTCM %d bytes\n", len);
            // Output all data except the 3 first bytes to Serial2
            if (len < 3) {
                //printf("Received RTCM message too short: %d bytes\n", len);
                digitalWrite(ERROR_LED, HIGH);
                delay(100);
                digitalWrite(ERROR_LED, LOW);
                return;
            }

            Serial2.write(data + 3, len - 3);  // Skip first 3 bytes (type, total parts, part index)
            Serial2.flush();                   // Ensure all data is sent immediately
            break;
        case MSG_TYPE_NMEA: {
            //printf("Base NMEA %d bytes\n", len);
            if (len < 4) {
                //printf("Received NMEA message too short: %d bytes\n", len);
                digitalWrite(ERROR_LED, HIGH);
                delay(100);
                digitalWrite(ERROR_LED, LOW);
                return;
            }

            String nmeaStr((const char *)(data + 3), len - 3);
            nmeaStr.trim();  // Remove any trailing whitespace
            printf("BASE: %s\n", nmeaStr.c_str());
            break;
        }
        default:
            printf("Unknown ESP-NOW message received: %02X\n", data[0]);
            digitalWrite(ERROR_LED, HIGH);
            delay(100);
            digitalWrite(ERROR_LED, LOW);

            break;
    }
}

void Rover::startPairing() {
    stopPairing();

    // addBroadcast();

    xTaskCreate(
        pairingTask,
        "pairTask",
        3000,
        this,
        1,
        &pairingTaskHandle);
}

void Rover::stopPairing() {
    if (pairingTaskHandle != nullptr) {
        vTaskDelete(pairingTaskHandle);
        pairingTaskHandle = nullptr;
        // removeBroadcast();
    }

    digitalWrite(LYNX_A_LED, LOW);
    digitalWrite(LYNX_B_LED, LOW);
}

void Rover::pairingTask(void *arg) {
    Rover *self = (Rover *)arg;
    self->paired = false;

    digitalWrite(LYNX_A_LED, HIGH);
    digitalWrite(LYNX_B_LED, HIGH);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (!self->paired) {
        // Wait 60 seconds for pairing requests
        xTaskDelayUntil(&xLastWakeTime, 60000 / portTICK_RATE_MS);

        break;
    }

    self->stopPairing();
}
