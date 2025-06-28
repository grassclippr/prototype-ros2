#ifndef UROS_CLIENT_H
#define UROS_CLIENT_H

#include <Stream.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include <vector>

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
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
    void subscribeToStateChange(void (*callback)(ClientState));

   private:
    static void urosTask(void *arg);
    static void timerCallback(rcl_timer_t *timer, int64_t last_call_time);
    void reportNewState(ClientState new_state);

    bool create_entities();
    void destroy_entities();

    ClientState state = WAITING_AGENT;
    std::vector<void (*)(ClientState)> state_change_callbacks;

    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rcl_timer_t timer;
};

#endif  // UROS_CLIENT_H