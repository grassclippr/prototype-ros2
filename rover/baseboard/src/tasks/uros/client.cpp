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
    printf("support init\n");
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    printf("node init\n");
    RCCHECK(rclc_node_init_default(&node, "baseboard", "", &support));

    // APP
    printf("app init\n");
    for (auto &callback : onCreateCallbacks) {
        callback(&node, &support);
    }

    // COM
    printf("executor init\n");
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    printf("executor init callbacks\n");
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
    printf("executor fini\n");
    if (rclc_executor_fini(&executor) != RCL_RET_OK) {
        printf("Failed to fini executor\n");
    }
    printf("node fini\n");
    if (rcl_node_fini(&node) != RCL_RET_OK) {
        printf("Failed to fini node\n");
    }
    printf("support fini\n");
    if (rclc_support_fini(&support) != RCL_RET_OK) {
        printf("Failed to fini support\n");
    }
}

void UrosClient::setup(Stream &stream) {
    // create a cobs cobs_encode function

    // create a custom Stream wrapper that applies COBS encoding to all outgoing data
    // set_microros_serial_transports(cobs_stream);

    rmw_uros_set_custom_transport(
        true,  // HLDC-like framing
        &cobs_stream,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read);

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
            printf("state %d\n", self->state);
        }

        switch (self->state) {
            case WAITING_AGENT:
                // Check every 500ms if agent is available
                if (rmw_uros_ping_agent(300, 1) == RMW_RET_OK) {
                    printf("ping success\n");
                    self->state = CONNECTING;
                } else {
                    printf("ping wait\n");
                    delay(500);
                }
                break;

            case CONNECTING:
                self->state = (self->create_entities()) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (self->state == AGENT_DISCONNECTED) {
                    printf("connect fail\n");
                }
                break;

            case AGENT_CONNECTED:
                static unsigned long last_ping = 0;
                // Check every 1000ms if agent is still connected
                if (millis() - last_ping > 1000) {
                    last_ping = millis();
                    if (rmw_uros_ping_agent(300, 3) != RMW_RET_OK) {
                        printf("ping fail\n");
                        self->state = AGENT_DISCONNECTED;
                    }
                }

                // if (rmw_uros_ping_agent(300, 3) == RMW_RET_OK) {
                if (rclc_executor_spin_some(&self->executor, RCL_MS_TO_NS(100)) != RCL_RET_OK) {
                    printf("spin fail\n");
                    self->state = AGENT_DISCONNECTED;
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);
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
