#include "./client.h"

#include <Arduino.h>

#include "./cobs_wrapper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

CobsStream cobs_stream(USBSerial);

void UrosClient::reportNewState(ClientState new_state) {
    state = new_state;
    for (auto &callback : state_change_callbacks) {
        callback(new_state);
    }
}

bool UrosClient::create_entities() {
    allocator = rcl_get_default_allocator();

    // COM
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "baseboard", "", &support));

    // APP
    for (auto &callback : onCreateCallbacks) {
        callback(&node, &support);
    }

    // COM
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    for (auto &callback : onExecutorInitCallbacks) {
        callback(&executor);
    }

    return true;
}

void UrosClient::destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // APP
    for (auto &callback : onDestroyCallbacks) {
        callback(&node, &support);
    }

    // COM
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void UrosClient::setup(Stream &stream) {
    // create a cobs cobs_encode function

    // create a custom Stream wrapper that applies COBS encoding to all outgoing data
    set_microros_serial_transports(cobs_stream);

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
