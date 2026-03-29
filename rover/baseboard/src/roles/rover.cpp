#include "./rover.h"

#include "tasks/uros/serial_mux_debug.h"
#include "tasks/uros/error.h"

#define printf serial_mux::debug_printf

#include <cmath>

#include "./espnow.h"

#ifndef SERIAL_MUX_HEARTBEAT
#define SERIAL_MUX_HEARTBEAT 1
#endif

#ifndef SERIAL_MUX_DISABLE_ROS
#define SERIAL_MUX_DISABLE_ROS 0
#endif

constexpr uint8_t DRIVE_ACK_ACCEPTED = 1;
constexpr uint8_t DRIVE_ACK_REJECTED = 2;
constexpr uint8_t DRIVE_ACK_TIMED_OUT = 3;

static Rover *selfRover = nullptr;

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
        if (!rover_msgs__msg__DriveCommand__init(&drive_command_msg)) {
            printf("Failed to init DriveCommand message\n");
            return false;
        }
        drive_command_msg_initialized = true;
        if (!rover_msgs__msg__DriveCommandAck__init(&drive_ack_msg)) {
            printf("Failed to init DriveCommandAck message\n");
            return false;
        }
        drive_ack_msg_initialized = true;

        rcl_ret_t rc = rclc_publisher_init_default(
            &drive_ack_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, DriveCommandAck),
            "/drive_command_ack");
        if (rc != RCL_RET_OK) {
            log_rcl_error("drive ack publisher init", rc);
            return false;
        }
        drive_ack_publisher_initialized = true;

        // Subscription creation can fail if the XRCE session isn't
        // fully ready yet.  Retry with a short delay to let the
        // session settle after the preceding publisher creation.
        constexpr int kMaxSubRetries = 3;
        constexpr int kSubRetryDelayMs = 500;
        for (int attempt = 1; attempt <= kMaxSubRetries; ++attempt) {
            if (attempt > 1) {
                printf("Retrying /drive_command subscription (attempt %d/%d)...\n",
                       attempt, kMaxSubRetries);
                delay(kSubRetryDelayMs);
                rcl_reset_error();
                drive_command_sub = rcl_get_zero_initialized_subscription();
            }
            rc = rclc_subscription_init_default(
                &drive_command_sub,
                node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(rover_msgs, msg, DriveCommand),
                "/drive_command");
            if (rc == RCL_RET_OK) {
                printf("/drive_command subscription OK (attempt %d)\n", attempt);
                break;
            }
            log_rcl_error("drive command subscription init", rc);
        }
        if (rc != RCL_RET_OK) {
            printf("/drive_command subscription failed after %d attempts\n", kMaxSubRetries);
            return false;
        }
        drive_command_sub_initialized = true;

        printf("Drive command entities created\n");
        return true;
    });
    uros_client.onExecutorInit([&](rclc_executor_t *executor) {
        RCCHECK(rclc_executor_add_subscription(
            executor,
            &drive_command_sub,
            &drive_command_msg,
            [](const void *msgin) -> void {
                auto *msg = static_cast<const rover_msgs__msg__DriveCommand *>(msgin);
                const char *detail = nullptr;
                if (!selfRover->validateDriveCommand(msg, &detail)) {
                    printf("Rejected drive command seq=%lu reason=%s\n",
                           static_cast<unsigned long>(msg->seq), detail);
                    selfRover->publishDriveAck(msg->seq, DRIVE_ACK_REJECTED, detail);
                    return;
                }

                printf("Accepted drive command seq=%lu linear_x=%.3f angular_z=%.3f timeout_ms=%lu\n",
                       static_cast<unsigned long>(msg->seq),
                       static_cast<double>(msg->linear_x),
                       static_cast<double>(msg->angular_z),
                       static_cast<unsigned long>(msg->timeout_ms));
                selfRover->motors.setCommand(msg->linear_x, msg->angular_z, msg->seq, msg->timeout_ms);
                selfRover->publishDriveAck(msg->seq, DRIVE_ACK_ACCEPTED, detail);
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
        if (drive_command_sub_initialized) {
            RCSOFTCHECK(rcl_subscription_fini(&drive_command_sub, node));
            drive_command_sub_initialized = false;
        }
        if (drive_ack_publisher_initialized) {
            RCSOFTCHECK(rcl_publisher_fini(&drive_ack_publisher, node));
            drive_ack_publisher_initialized = false;
        }
        if (drive_ack_msg_initialized) {
            rover_msgs__msg__DriveCommandAck__fini(&drive_ack_msg);
            drive_ack_msg_initialized = false;
        }
        if (drive_command_msg_initialized) {
            rover_msgs__msg__DriveCommand__fini(&drive_command_msg);
            drive_command_msg_initialized = false;
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
    self->sendNmeaCommand("PAIR062,0,1");

    self->nmea_msg.sentence.data = (char *)malloc(82 + 1 * sizeof(char));
    self->nmea_msg.sentence.size = 0;
    self->nmea_msg.sentence.capacity = 82 + 1;  // 82 is the maximum length of NMEA sentences, +1 for null terminator

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
                String line = Serial2.readStringUntil('\n');
                if (line.length() == 0) {
                    printf("Error reading NMEA line\n");
                    break;
                }

                // printf("%s\n", line.c_str());

                if (line.length() <= 82 && self->uros_client.isConnected() && self->nmea_publisher_initialized) {
                    memcpy(selfRover->nmea_msg.sentence.data, line.c_str(), line.length());
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

        uint32_t expired_seq = 0;
        if (self->motors.expireIfTimedOut(millis(), &expired_seq)) {
            self->publishDriveAck(expired_seq, DRIVE_ACK_TIMED_OUT, "command timed out");
        }
    }
}

void Rover::heartbeatTask(void *arg) {
    Rover *self = (Rover *)arg;
    (void)self;

    uint32_t counter = 0;
    while (true) {
        printf("heartbeat %lu\n", static_cast<unsigned long>(counter));
        counter++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

bool Rover::validateDriveCommand(const rover_msgs__msg__DriveCommand *msg, const char **detail) const {
    if (!std::isfinite(msg->linear_x) || !std::isfinite(msg->angular_z)) {
        *detail = "non-finite velocity";
        return false;
    }

    if (msg->timeout_ms == 0 || msg->timeout_ms > 5000) {
        *detail = "timeout_ms out of range";
        return false;
    }

    *detail = "accepted";
    return true;
}

void Rover::publishDriveAck(uint32_t seq, uint8_t status, const char *detail) {
    if (!uros_client.isConnected()) {
        return;
    }

    uint32_t now_ms = millis();
    drive_ack_msg.stamp.sec = now_ms / 1000;
    drive_ack_msg.stamp.nanosec = (now_ms % 1000) * 1000000UL;
    drive_ack_msg.seq = seq;
    drive_ack_msg.status = status;
    rosidl_runtime_c__String__assign(&drive_ack_msg.detail, detail);
    RCSOFTCHECK(rcl_publish(&drive_ack_publisher, &drive_ack_msg, NULL));
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
            //printf("RTCM %d bytes\n", len);
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
            //printf("NMEA %d bytes\n", len);
            if (len < 4) {
                //printf("Received NMEA message too short: %d bytes\n", len);
                digitalWrite(ERROR_LED, HIGH);
                delay(100);
                digitalWrite(ERROR_LED, LOW);
                return;
            }

            String nmeaStr((const char *)(data + 3), len - 3);
            // nmeaStr.trim();  // Remove any trailing whitespace
            //printf("%s\n", nmeaStr.c_str());
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
