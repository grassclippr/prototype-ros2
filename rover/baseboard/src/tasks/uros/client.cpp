#include <Arduino.h>

#include "./client.h"
#ifndef SERIAL_MUX_ENABLE
#define SERIAL_MUX_ENABLE 1
#endif

#ifndef SERIAL_MUX_PACKET_MODE
#define SERIAL_MUX_PACKET_MODE 1
#endif
#if SERIAL_MUX_ENABLE
#include "./serial_mux.h"
#include "./serial_mux_debug.h"
#include "./serial_mux_transport.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void UrosClient::reportNewState(ClientState new_state) {
    state = new_state;
    for (auto &callback : state_change_callbacks) {
        callback(new_state);
    }
}

#ifndef SERIAL_MUX_SKIP_PING
#define SERIAL_MUX_SKIP_PING 0
#endif

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

void UrosClient::setup(Stream & stream) {
    #if SERIAL_MUX_ENABLE
    static serial_mux::SerialMux mux(stream);
    serial_mux::set_debug_mux(&mux);

    rmw_uros_set_custom_transport(
        SERIAL_MUX_PACKET_MODE ? false : true,
        &mux,
        serial_mux_transport_open,
        serial_mux_transport_close,
        serial_mux_transport_write,
        serial_mux_transport_read);
    #else
    set_microros_serial_transports(stream);
    #endif

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
                #if SERIAL_MUX_SKIP_PING
                self->state = CONNECTING;
                #else
                // Check every 500ms if agent is available
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    self->state = CONNECTING;
                } else {
                    delay(500);
                }
                #endif
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
