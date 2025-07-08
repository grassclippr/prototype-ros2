#include "./rover.h"
#include "./espnow.h"

constexpr float GEARBOX_RATIO = 30.0f;   // Example: 30:1 gearbox
constexpr float WHEEL_RADIUS_M = 0.05f;  // 5 cm wheel radius
constexpr float WHEEL_BASE_M = 0.20f;    // 20 cm distance between wheels

static Rover * selfRover = nullptr;

Rover::Rover() {
    selfRover = this;

    USBSerial.begin(115200);
    leds.setup();
    motors.setup();
    uros_client.setup(USBSerial);

    uros_client.subscribeToStateChange([&](ClientState state) {
        if (state == AGENT_CONNECTED) {
            leds.status_led.on();
        } else {
            leds.status_led.blink1();
        }
    });
    uros_client.onCreateEntities([&](rcl_node_t *node, rclc_support_t *support) {
        msg.data = 0;

        rclc_publisher_init_default(
            &publisher,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/baseboard");

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
        RCCHECK(rclc_executor_add_timer(executor, &timer));

        RCCHECK(rclc_executor_add_subscription(
            executor,
            &cmd_vel_sub,
            &cmd_vel_msg,
            [](const void *msgin) {
                auto *msg = (const geometry_msgs__msg__Twist *)msgin;
                //motors.setVelocities(msg);

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

                // Example: set LEDs based on direction
                digitalWrite(LYNX_A_LED, left_motor_angular = 0 ? HIGH : LOW);
                digitalWrite(LYNX_B_LED, left_motor_angular > 0 ? HIGH : LOW);
                digitalWrite(LYNX_C_LED, right_motor_angular = 0 ? HIGH : LOW);
                digitalWrite(LYNX_D_LED, right_motor_angular > 0 ? HIGH : LOW);
            },
            ON_NEW_DATA));
    });

    // Cleanup when connection is lost
    uros_client.onDestroyEntities([&](rcl_node_t *node, rclc_support_t *support) {
        rcl_publisher_fini(&publisher, node);
        rcl_timer_fini(&timer);
        rcl_subscription_fini(&cmd_vel_sub, node);
    });

    // Initialize ESP-NOW
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
            selfRover->onEspNowRecv(mac, data, len);
        });
        //esp_now_register_send_cb(onEspNowSent);
        
        // Make sure to load the stored MAC from NVS
        loadPeerFromNVS();
    }
};

void Rover::onEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, size_t len) {
    if (len < 1) {
        return;
    }

    switch(data[0]) {
        case MSG_TYPE_PAIR_REQ: {
            addEspNowPeer(mac_addr);

            uint8_t ack[] = { MSG_TYPE_PAIR_ACK }; // Answer pair request with ACK
            esp_now_send(mac_addr, ack, sizeof(ack));
            break;
        }
        default:
            printf("Unknown ESP-NOW message received: %02X\n", data[0]);
            break;
    }
}
