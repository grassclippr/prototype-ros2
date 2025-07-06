#include <Arduino.h>

#include "tasks/leds/task.h"
#include "tasks/motors/task.h"
#include "tasks/uros/client.h"

constexpr float GEARBOX_RATIO = 30.0f;   // Example: 30:1 gearbox
constexpr float WHEEL_RADIUS_M = 0.05f;  // 5 cm wheel radius
constexpr float WHEEL_BASE_M = 0.20f;    // 20 cm distance between wheels

UrosClient uros_client;
LedControl leds;
MotorControl motors;

rcl_timer_t timer;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

void setup() {
    USBSerial.begin(115200);

    pinMode(LYNX_A_LED, OUTPUT);
    pinMode(LYNX_B_LED, OUTPUT);
    pinMode(LYNX_C_LED, OUTPUT);
    pinMode(LYNX_D_LED, OUTPUT);

    leds.setup();
    motors.setup();
    uros_client.setup(USBSerial);

    uros_client.subscribeToStateChange([](ClientState state) {
        if (state == AGENT_CONNECTED) {
            leds.status_led.on();
        } else {
            leds.status_led.blink1();
        }

        /*digitalWrite(LYNX_A_LED, state == WAITING_AGENT ? HIGH : LOW);
        digitalWrite(LYNX_B_LED, state == CONNECTING ? HIGH : LOW);
        digitalWrite(LYNX_C_LED, state == AGENT_CONNECTED ? HIGH : LOW);
        digitalWrite(LYNX_D_LED, state == AGENT_DISCONNECTED ? HIGH : LOW);*/
        // delay(100);
    });
    uros_client.onCreateEntities([](rcl_node_t *node, rclc_support_t *support) {
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
                RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                msg.data++;
            },
            true);  // autostart = true
    });
    uros_client.onExecutorInit([](rclc_executor_t *executor) {
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
    uros_client.onDestroyEntities([](rcl_node_t *node, rclc_support_t *support) {
        rcl_publisher_fini(&publisher, node);
        rcl_timer_fini(&timer);
        rcl_subscription_fini(&cmd_vel_sub, node);
    });
}

void loop() {
    vTaskDelete(NULL);
}
