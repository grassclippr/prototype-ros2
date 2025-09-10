#include "./rover.h"

#include "./espnow.h"

constexpr float GEARBOX_RATIO = 30.0f;   // Example: 30:1 gearbox
constexpr float WHEEL_RADIUS_M = 0.05f;  // 5 cm wheel radius
constexpr float WHEEL_BASE_M = 0.20f;    // 20 cm distance between wheels

static Rover *selfRover = nullptr;

Rover::Rover() {
    selfRover = this;

    USBSerial.begin(115200);
    leds.setup();
    motors.setup();
    uros_client.setup(USBSerial);

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
            // leds.status_led.on();
        } else {
            digitalWrite(STATUS_LED, LOW);
            // leds.status_led.blink1();
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

        rclc_publisher_init_default(
            &publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "baseboard");

        /*rclc_publisher_init_default(
            &nmea_publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nmea_msgs, msg, Sentence),
            "nmea_sentence");*/

        rclc_subscription_init_default(
            &cmd_vel_sub,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel");

        const uint32_t timer_timeout = 1000;  // Set to desired timeout in ms
        rclc_timer_init_default2(
            &timer,
            support,
            RCL_MS_TO_NS(timer_timeout),
            [](rcl_timer_t *timer, int64_t last_call_time) {
                RCSOFTCHECK(rcl_publish(&selfRover->publisher, &selfRover->msg, NULL));
                selfRover->msg.data++;
            },
            true);  // autostart = true
    });
    uros_client.onExecutorInit([&](rclc_executor_t *executor) {
        // Add timer to executor
        // RCCHECK(rclc_executor_add_timer(executor, &timer));

        RCCHECK(rclc_executor_add_subscription(
            executor,
            &cmd_vel_sub,
            &cmd_vel_msg,
            [](const void *msgin) -> void {
                auto *msg = (const geometry_msgs__msg__Twist *)msgin;
                // motors.setVelocities(msg);

                // Calculate wheel speeds (m/s)
                float v = msg->linear.x;   // Linear velocity (m/s)
                float w = msg->angular.z;  // Angular velocity (rad/s)

                // Differential drive kinematics
                float left_wheel_speed = v - (w * WHEEL_BASE_M / 2.0f);   // m/s
                float right_wheel_speed = v + (w * WHEEL_BASE_M / 2.0f);  // m/s

                // Convert to wheel angular speed (rad/s)
                float left_wheel_angular = left_wheel_speed / WHEEL_RADIUS_M;
                float right_wheel_angular = right_wheel_speed / WHEEL_RADIUS_M;

                // Apply gearbox ratio to get motor shaft speed (rad/s)
                float left_motor_angular = left_wheel_angular * GEARBOX_RATIO;
                float right_motor_angular = right_wheel_angular * GEARBOX_RATIO;

                printf("Cmd Vel: v=%.2f m/s, w=%.2f rad/s\n", v, w);
                // Example: set LEDs based on direction
                // digitalWrite(LYNX_A_LED, left_motor_angular = 0 ? HIGH : LOW);
                // digitalWrite(LYNX_B_LED, left_motor_angular > 0 ? HIGH : LOW);
                // digitalWrite(LYNX_C_LED, right_motor_angular = 0 ? HIGH : LOW);
                // digitalWrite(LYNX_D_LED, right_motor_angular > 0 ? HIGH : LOW);
                return;
            },
            ON_NEW_DATA));
        return true;
    });

    // Cleanup when connection is lost
    uros_client.onDestroyEntities([&](rcl_node_t *node, rclc_support_t *support) {
        RCSOFTCHECK(rcl_publisher_fini(&publisher, node));
        RCSOFTCHECK(rcl_timer_fini(&timer));
        RCSOFTCHECK(rcl_subscription_fini(&cmd_vel_sub, node));
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

                if (line.length() <= 82 && self->uros_client.isConnected()) {
                    /*memcpy(selfRover->nmea_msg.sentence.data, line.c_str(), line.length());
                    selfRover->nmea_msg.sentence.size = line.length();

                    RCSOFTCHECK(rcl_publish(&selfRover->nmea_publisher, &selfRover->nmea_msg, NULL));*/
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
            // printf("RTCM %d bytes\n", len);
            //  Output all data except the 3 first bytes to Serial2
            if (len < 3) {
                // printf("Received RTCM message too short: %d bytes\n", len);
                digitalWrite(ERROR_LED, HIGH);
                delay(100);
                digitalWrite(ERROR_LED, LOW);
                return;
            }

            Serial2.write(data + 3, len - 3);  // Skip first 3 bytes (type, total parts, part index)
            Serial2.flush();                   // Ensure all data is sent immediately
            break;
        case MSG_TYPE_NMEA: {
            // printf("NMEA %d bytes\n", len);
            if (len < 4) {
                // printf("Received NMEA message too short: %d bytes\n", len);
                digitalWrite(ERROR_LED, HIGH);
                delay(100);
                digitalWrite(ERROR_LED, LOW);
                return;
            }

            String nmeaStr((const char *)(data + 3), len - 3);
            // nmeaStr.trim();  // Remove any trailing whitespace
            // printf("%s\n", nmeaStr.c_str());
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

void Rover::sendDebugMessage(const String &message) {
    sendDebugMessage(message.c_str());
}

void Rover::sendDebugMessage(const char *message) {
    USBSerial.write(DEBUG_MAGIC_BYTE);
    USBSerial.print(message);
    USBSerial.print("\n");
    USBSerial.flush();
}
