#ifndef UROS_CLIENT_H
#define UROS_CLIENT_H

#include <Stream.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <functional>
#include <vector>

#include "./error.h"

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            log_rcl_error(#fn, temp_rc); \
            return false;              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }

enum ClientState {
    WAITING_AGENT,
    CONNECTING,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

class UrosClient {
   public:
    void setup(Stream &stream);

    void subscribeToStateChange(std::function<void(ClientState)> callback) {
        state_change_callbacks.push_back(callback);
    }
    void onCreateEntities(std::function<bool(rcl_node_t *node, rclc_support_t *support)> callback) {
        onCreateCallbacks.push_back(callback);
    }
    void onExecutorInit(std::function<bool(rclc_executor_t *executor)> callback) {
        onExecutorInitCallbacks.push_back(callback);
    }
    void onDestroyEntities(std::function<void(rcl_node_t *node, rclc_support_t *support)> callback) {
        onDestroyCallbacks.push_back(callback);
    }

    bool isConnected() const {
        return state == AGENT_CONNECTED;
    }

   private:
    static void urosTask(void *arg);
    static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
    void reportNewState(ClientState new_state);

    bool create_entities();
    void destroy_entities();

    ClientState state = WAITING_AGENT;
    std::vector<std::function<void(ClientState)>> state_change_callbacks;
    std::vector<std::function<bool(rcl_node_t *node, rclc_support_t *support)>> onCreateCallbacks;
    std::vector<std::function<bool(rclc_executor_t *executor)>> onExecutorInitCallbacks;
    std::vector<std::function<void(rcl_node_t *node, rclc_support_t *support)>> onDestroyCallbacks;

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_support_t support{};
    rcl_allocator_t allocator;
    rcl_node_t node = rcl_get_zero_initialized_node();
    bool support_initialized = false;
    bool node_initialized = false;
    bool executor_initialized = false;
    uint32_t last_ping_ms = 0;
};

#endif  // UROS_CLIENT_H
