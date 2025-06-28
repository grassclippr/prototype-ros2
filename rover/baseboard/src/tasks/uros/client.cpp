#include <Arduino.h>
#include "./client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void UrosClient::subscribeToStateChange(void (*callback)(ClientState)) {
    state_change_callbacks.push_back(callback);
}

void UrosClient::reportNewState(ClientState new_state) {
    state = new_state;
    for (auto &callback : state_change_callbacks) {
        callback(new_state);
    }
}

void UrosClient::timerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
}

bool UrosClient::create_entities() {
    allocator = rcl_get_default_allocator();

    // COM
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "baseboard", "", &support));

    // APP
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/baseboard"));

    // Define timer_timeout in milliseconds
    const uint32_t timer_timeout = 1000;  // Set to desired timeout in ms
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timerCallback,
        true));  // autostart = true

    // COM
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;
    return true;
}

void UrosClient::destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // APP
    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);

    // COM
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void UrosClient::setup(Stream & stream) {
    set_microros_serial_transports(stream);

    xTaskCreate(
        urosTask,
        "urosTask",
        3000,
        this,
        1,
        NULL);
}

void UrosClient::urosTask(void *arg) {
    UrosClient *self = static_cast<UrosClient *>(arg);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);

        static int previous_state = AGENT_DISCONNECTED;
        if (previous_state != self->state) {
            self->reportNewState(self->state);
            previous_state = self->state;
        }

        switch (self->state) {
            case WAITING_AGENT:
                // Check every 500ms if agent is available
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    self->state = CONNECTING;
                } else {
                    delay(500);
                }
                break;

            case CONNECTING:
                self->state = (self->create_entities()) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                break;

            case AGENT_CONNECTED:
                // Check every 200ms if agent is still connected
                if (rmw_uros_ping_agent(100, 3) == RMW_RET_OK) {
                    rclc_executor_spin_some(&self->executor, RCL_MS_TO_NS(100));
                } else {
                    self->state = AGENT_DISCONNECTED;
                }
                delay(200);
                break;

            case AGENT_DISCONNECTED:
                self->destroy_entities();
                self->state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }
}
