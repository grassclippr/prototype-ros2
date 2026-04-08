#include "./rover.h"

#include "tasks/uros/error.h"

#include <cmath>

#include "./espnow.h"

#ifndef SERIAL_MUX_HEARTBEAT
#define SERIAL_MUX_HEARTBEAT 1
#endif

#ifndef SERIAL_MUX_DISABLE_ROS
#define SERIAL_MUX_DISABLE_ROS 0
#endif

constexpr uint32_t WHEEL_CMD_TIMEOUT_MS = 500;
constexpr uint32_t GNSS_READ_TIMEOUT_MS = 200;
constexpr size_t MAX_NMEA_SENTENCE_LEN = 82;
constexpr size_t WHEEL_CMD_ELEMENT_COUNT = 2;

static Rover *selfRover = nullptr;

namespace {
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

        line += ch;
        deadline = millis() + timeout_ms;
        if (line.length() > MAX_NMEA_SENTENCE_LEN) {
            discardUntilNewline(serial, timeout_ms);
            return false;
        }
    }
}

bool initWheelCommandMessage(std_msgs__msg__Float32MultiArray &msg) {
    if (!std_msgs__msg__Float32MultiArray__init(&msg)) {
        return false;
    }

    msg.layout.dim.data = nullptr;
    msg.layout.dim.size = 0;
    msg.layout.dim.capacity = 0;
    msg.layout.data_offset = 0;
    msg.data.data = static_cast<float *>(malloc(sizeof(float) * WHEEL_CMD_ELEMENT_COUNT));
    if (msg.data.data == nullptr) {
        std_msgs__msg__Float32MultiArray__fini(&msg);
        return false;
    }
    memset(msg.data.data, 0, sizeof(float) * WHEEL_CMD_ELEMENT_COUNT);
    msg.data.size = WHEEL_CMD_ELEMENT_COUNT;
    msg.data.capacity = WHEEL_CMD_ELEMENT_COUNT;
    return true;
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
        wheel_cmd_sub = rcl_get_zero_initialized_subscription();
        if (!initWheelCommandMessage(wheel_cmd_msg)) {
            printf("Failed to initialize wheel command message buffer\n");
            return false;
        }
        wheel_cmd_msg_initialized = true;

        rcl_ret_t rc = rclc_publisher_init_default(
            &publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "baseboard");
        if (rc != RCL_RET_OK) {
            log_rcl_error("baseboard publisher init", rc);
            return false;
        }
        baseboard_publisher_initialized = true;

        rc = rclc_publisher_init_default(
            &nmea_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nmea_msgs, msg, Sentence),
            "nmea_sentence");
        if (rc != RCL_RET_OK) {
            log_rcl_error("nmea publisher init", rc);
            return false;
        }
        nmea_publisher_initialized = true;

        // Subscription creation can fail if the XRCE session isn't
        // fully ready yet.  Retry with a short delay to let the
        // session settle.
        constexpr int kMaxSubRetries = 3;
        constexpr int kSubRetryDelayMs = 500;
        rc = RCL_RET_ERROR;
        for (int attempt = 1; attempt <= kMaxSubRetries; ++attempt) {
            if (attempt > 1) {
                delay(kSubRetryDelayMs);
                rcl_reset_error();
                wheel_cmd_sub = rcl_get_zero_initialized_subscription();
            }
            rc = rclc_subscription_init_default(
                &wheel_cmd_sub,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                "/wheel_cmd");
            if (rc == RCL_RET_OK) {
                break;
            }
            log_rcl_error("wheel_cmd subscription init", rc);
        }
        if (rc != RCL_RET_OK) {
            return false;
        }
        wheel_cmd_sub_initialized = true;

        const uint32_t timer_timeout = 1000;
        rc = rclc_timer_init_default2(
            &timer,
            support,
            RCL_MS_TO_NS(timer_timeout),
            [](rcl_timer_t *timer, int64_t last_call_time) {
                (void)timer;
                (void)last_call_time;
                RCSOFTCHECK(rcl_publish(&selfRover->publisher, &selfRover->msg, NULL));
                selfRover->msg.data++;
            },
            true);
        if (rc != RCL_RET_OK) {
            log_rcl_error("baseboard timer init", rc);
            return false;
        }
        timer_initialized = true;

        // Wheel velocity publisher (measured from encoders)
        odom_vel_publisher = rcl_get_zero_initialized_publisher();
        geometry_msgs__msg__Twist__init(&odom_vel_msg);
        rc = rclc_publisher_init_default(
            &odom_vel_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/wheel_velocities");
        if (rc != RCL_RET_OK) {
            log_rcl_error("odom_vel publisher init", rc);
            return false;
        }
        odom_vel_publisher_initialized = true;

        // 20 Hz timer to publish encoder feedback
        odom_vel_timer = rcl_get_zero_initialized_timer();
        rc = rclc_timer_init_default2(
            &odom_vel_timer,
            support,
            RCL_MS_TO_NS(50),  // 50ms = 20Hz
            [](rcl_timer_t *timer, int64_t last_call_time) {
                (void)timer;
                (void)last_call_time;
                auto left  = selfRover->motors.getLeftWheel();
                auto right = selfRover->motors.getRightWheel();

                selfRover->odom_vel_msg.linear.x = (left.velocity_mps + right.velocity_mps) * 0.5;
                selfRover->odom_vel_msg.angular.z = (right.velocity_mps - left.velocity_mps) / TRACK_WIDTH_METERS;
                selfRover->odom_vel_msg.linear.y = left.velocity_mps;   // per-wheel debug
                selfRover->odom_vel_msg.linear.z = right.velocity_mps;  // per-wheel debug

                RCSOFTCHECK(rcl_publish(&selfRover->odom_vel_publisher, &selfRover->odom_vel_msg, NULL));
            },
            true);
        if (rc != RCL_RET_OK) {
            log_rcl_error("odom_vel timer init", rc);
            return false;
        }
        odom_vel_timer_initialized = true;

        return true;
    });
    uros_client.onExecutorInit([&](rclc_executor_t *executor) {
        RCCHECK(rclc_executor_add_timer(executor, &timer));
        RCCHECK(rclc_executor_add_timer(executor, &odom_vel_timer));
        RCCHECK(rclc_executor_add_subscription(
            executor,
            &wheel_cmd_sub,
            &wheel_cmd_msg,
            [](const void *msgin) -> void {
                auto *wheel_cmd = static_cast<const std_msgs__msg__Float32MultiArray *>(msgin);

                if (wheel_cmd->data.size < WHEEL_CMD_ELEMENT_COUNT) {
                    printf("Rejected wheel_cmd: expected %u elements, got %u\n",
                           static_cast<unsigned>(WHEEL_CMD_ELEMENT_COUNT),
                           static_cast<unsigned>(wheel_cmd->data.size));
                    return;
                }

                const float left = wheel_cmd->data.data[0];
                const float right = wheel_cmd->data.data[1];
                if (!std::isfinite(left) || !std::isfinite(right)) {
                    printf("Rejected wheel_cmd: non-finite velocity\n");
                    return;
                }

                selfRover->motors.setWheelCommand(left, right, 0, WHEEL_CMD_TIMEOUT_MS);
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
        if (nmea_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&nmea_publisher, node));
            nmea_publisher_initialized = false;
        }
        if (wheel_cmd_sub_initialized) {
            RCSOFTCHECK(rcl_subscription_fini(&wheel_cmd_sub, node));
            wheel_cmd_sub_initialized = false;
        }
        if (wheel_cmd_msg_initialized) {
            std_msgs__msg__Float32MultiArray__fini(&wheel_cmd_msg);
            wheel_cmd_msg_initialized = false;
        }
        if (odom_vel_timer_initialized) {
            RCSOFTCHECK(rcl_timer_fini(&odom_vel_timer));
            odom_vel_timer_initialized = false;
        }
        if (odom_vel_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&odom_vel_publisher, node));
            odom_vel_publisher_initialized = false;
        }
        if (odom_vel_timer_initialized) {
            RCSOFTCHECK(rcl_timer_fini(&odom_vel_timer));
            odom_vel_timer_initialized = false;
        }
        if (odom_vel_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&odom_vel_publisher, node));
            geometry_msgs__msg__Twist__fini(&odom_vel_msg);
            odom_vel_publisher_initialized = false;
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

void Rover::gnssReceiveTask(void *arg) {
    Rover *self = (Rover *)arg;

    // start uart port with UART_RX_PIN and UART_TX_PIN
    Serial2.begin(460800, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    // Serial2.print("$PQTMCFGMSGRATE,W,GGA,1,1*58\r\n");
    // Serial2.print("$PAIR062,0,1*3F\r\n");
    self->sendNmeaCommand("PQTMCFGRCVRMODE,W,1"); // Set receiver to rover mode (accept RTCM corrections)
    delay(100);
    self->sendNmeaCommand("PAIR062,0,1");
    delay(30);
    self->sendNmeaCommand("QTMSAVEPAR"); // Save settings to non-volatile memory, so they persist after reboot
    delay(100);

    self->nmea_msg.sentence.data = (char *)malloc(82 + 1 * sizeof(char));
    self->nmea_msg.sentence.size = 0;
    self->nmea_msg.sentence.capacity = 82 + 1;  // 82 is the maximum length of NMEA sentences, +1 for null terminator

    uint32_t last_status_print_ms = 0;

    while (true) {
        if (Serial2.available() < 1) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
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

                if (line.length() <= MAX_NMEA_SENTENCE_LEN && self->uros_client.isConnected() && self->nmea_publisher_initialized) {
                    memset(selfRover->nmea_msg.sentence.data, 0, selfRover->nmea_msg.sentence.capacity);
                    memcpy(selfRover->nmea_msg.sentence.data, line.c_str(), line.length());
                    selfRover->nmea_msg.sentence.data[line.length()] = '\0';
                    selfRover->nmea_msg.sentence.size = line.length();

                    RCSOFTCHECK(rcl_publish(&selfRover->nmea_publisher, &selfRover->nmea_msg, NULL));
                }
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
