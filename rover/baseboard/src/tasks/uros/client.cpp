#include <Arduino.h>

#include "./client.h"
#include <rmw_microros/error_handling.h>
#ifndef SERIAL_MUX_ENABLE
#define SERIAL_MUX_ENABLE 1
#endif

#ifndef SERIAL_MUX_PACKET_MODE
#define SERIAL_MUX_PACKET_MODE 1
#endif
#if SERIAL_MUX_ENABLE
#include "./serial_mux.h"
#include "./serial_mux_transport.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef UROS_STACK_MONITOR
#define UROS_STACK_MONITOR 1
#endif

#if defined(RMW_UROS_ERROR_HANDLING)
namespace {
const char *entity_to_string(rmw_uros_error_entity_type_t entity) {
    switch (entity) {
        case RMW_UROS_ERROR_ON_UNKNOWN: return "unknown";
        case RMW_UROS_ERROR_ON_NODE: return "node";
        case RMW_UROS_ERROR_ON_SERVICE: return "service";
        case RMW_UROS_ERROR_ON_CLIENT: return "client";
        case RMW_UROS_ERROR_ON_SUBSCRIPTION: return "subscription";
        case RMW_UROS_ERROR_ON_PUBLISHER: return "publisher";
        case RMW_UROS_ERROR_ON_GRAPH: return "graph";
        case RMW_UROS_ERROR_ON_GUARD_CONDITION: return "guard_condition";
        case RMW_UROS_ERROR_ON_TOPIC: return "topic";
        default: return "invalid";
    }
}

const char *source_to_string(rmw_uros_error_source_t source) {
    switch (source) {
        case RMW_UROS_ERROR_ENTITY_CREATION: return "entity_creation";
        case RMW_UROS_ERROR_ENTITY_DESTRUCTION: return "entity_destruction";
        case RMW_UROS_ERROR_CHECK: return "check";
        case RMW_UROS_ERROR_NOT_IMPLEMENTED: return "not_implemented";
        case RMW_UROS_ERROR_MIDDLEWARE_ALLOCATION: return "middleware_allocation";
        default: return "invalid";
    }
}

void uros_error_callback(
    const rmw_uros_error_entity_type_t entity,
    const rmw_uros_error_source_t source,
    const rmw_uros_error_context_t context,
    const char *file,
    const int line)
{
    printf(
        "rmw_uros error entity=%s source=%s desc=%s topic=%s node=%s file=%s line=%d\n",
        entity_to_string(entity),
        source_to_string(source),
        context.description ? context.description : "<null>",
        context.topic_name ? context.topic_name : "<null>",
        context.node ? context.node : "<null>",
        file ? file : "<null>",
        line);
}
}  // namespace
#endif

void UrosClient::reportNewState(ClientState new_state) {
    state = new_state;
    for (auto &callback : state_change_callbacks) {
        callback(new_state);
    }
}

void UrosClient::requestReconnect(const char *reason) {
    if (state != AGENT_CONNECTED) {
        return;
    }

    if (reason != nullptr && reason[0] != '\0') {
        printf("uros reconnect requested: %s\n", reason);
    } else {
        printf("uros reconnect requested\n");
    }
    state = AGENT_DISCONNECTED;
}

#ifndef SERIAL_MUX_SKIP_PING
#define SERIAL_MUX_SKIP_PING 0
#endif

bool UrosClient::create_entities() {
    allocator = rcl_get_default_allocator();

    // COM
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    support_initialized = true;
    RCCHECK(rclc_node_init_default(&node, "baseboard", "", &support));
    node_initialized = true;

    // Allow the XRCE session to settle after node creation.
    // Without this, the first publisher/subscription creation often
    // fails because uxr_run_session_until_all_status() times out.
    vTaskDelay(pdMS_TO_TICKS(3000));

    // APP
    for (auto &callback : onCreateCallbacks) {
        if (!callback(&node, &support)) {
            return false;
        }
    }

    // COM
    size_t executor_handle_count = 0;
    for (auto &registration : onExecutorInitCallbacks) {
        executor_handle_count += registration.handle_count;
    }
    RCCHECK(rclc_executor_init(&executor, &support.context, executor_handle_count, &allocator));
    executor_initialized = true;
    for (auto &registration : onExecutorInitCallbacks) {
        if (!registration.callback(&executor)) {
            return false;
        }
    }
    
    return true;
}

void UrosClient::destroy_entities() {
    if (support_initialized) {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    }

    // APP
    for (auto &callback : onDestroyCallbacks) {
        callback(&node, &support);
    }

    // COM
    if (executor_initialized) {
        RCSOFTCHECK(rclc_executor_fini(&executor));
        executor_initialized = false;
    }
    if (node_initialized) {
        RCSOFTCHECK(rcl_node_fini(&node));
        node_initialized = false;
    }
    if (support_initialized) {
        RCSOFTCHECK(rclc_support_fini(&support));
        support_initialized = false;
    }
}

void UrosClient::setup(Stream & stream) {
    #if SERIAL_MUX_ENABLE
    static serial_mux::SerialMux mux(stream);

    rmw_uros_set_custom_transport(
        SERIAL_MUX_PACKET_MODE ? MICROROS_TRANSPORTS_PACKET_MODE : MICROROS_TRANSPORTS_FRAMING_MODE,
        &mux,
        serial_mux_transport_open,
        serial_mux_transport_close,
        serial_mux_transport_write,
        serial_mux_transport_read);
    #else
    set_microros_serial_transports(stream);
    #endif

    #if defined(RMW_UROS_ERROR_HANDLING)
    rmw_uros_set_error_handling_callback(uros_error_callback);
    #endif

    xTaskCreate(
        urosTask,
        "urosTask",
        8192,
        this,
        1,
        NULL);
}

void UrosClient::urosTask(void *arg) {
    UrosClient *self = static_cast<UrosClient *>(arg);

    TickType_t xLastWakeTime = xTaskGetTickCount();
#if UROS_STACK_MONITOR
    TickType_t lastStackReport = 0;
    constexpr TickType_t kStackReportIntervalMs = 60000;
#endif
    while (1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);

#if UROS_STACK_MONITOR
        TickType_t now = xTaskGetTickCount();
        if ((now - lastStackReport) * portTICK_RATE_MS >= kStackReportIntervalMs) {
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            printf("uros stack hwm=%lu words\n",
                   static_cast<unsigned long>(hwm));
            lastStackReport = now;
        }
#endif

        static int previous_state = AGENT_DISCONNECTED;
        if (previous_state != self->state) {
            printf("uros state -> %d\n", self->state);
            self->reportNewState(self->state);
            previous_state = self->state;
        }

        switch (self->state) {
            case WAITING_AGENT:
            {
                #if SERIAL_MUX_SKIP_PING
                self->state = CONNECTING;
                #else
                // Check every 500ms if agent is available
                rcl_ret_t ping_rc = rmw_uros_ping_agent(100, 1);
                if (ping_rc == RMW_RET_OK) {
                    self->state = CONNECTING;
                } else {
                    delay(500);
                }
                #endif
                break;
            }

            case CONNECTING:
                self->state = (self->create_entities()) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                break;

            case AGENT_CONNECTED:
            {
                // Ping the agent infrequently to detect disconnects.
                if (millis() - self->last_ping_ms > 2000) {
                    self->last_ping_ms = millis();
                    rcl_ret_t ping_rc = rmw_uros_ping_agent(100, 1);
                    if (ping_rc != RMW_RET_OK) {
                        self->state = AGENT_DISCONNECTED;
                        break;
                    }
                }
                rclc_executor_spin_some(&self->executor, RCL_MS_TO_NS(10));
                delay(10);
                break;
            }

            case AGENT_DISCONNECTED:
                self->destroy_entities();
                self->state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }
}
