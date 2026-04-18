#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include <stdint.h>
#include <stddef.h>

#include "driver/ledc.h"
#include "driver/pcnt.h"

// --- Configurable encoder / control constants ---
constexpr float TICKS_PER_METER       = 150.0f;   // ~3 ticks ≈ 2cm → calibrate!
constexpr float TRACK_WIDTH_METERS    = 0.66f;
constexpr float PI_KP                 = 0.2f;
constexpr float PI_KI                 = 0.5f;
constexpr float PI_INTEGRAL_LIMIT     = 1.0f;
constexpr float CONTROL_PERIOD_S      = 0.01f;     // 10ms task period
constexpr size_t VEL_AVG_WINDOW       = 10;         // 100ms averaging window

template <size_t N>
struct MovingAverage {
    float buf[N] = {};
    size_t idx = 0;
    size_t count = 0;

    void push(float v) {
        buf[idx] = v;
        idx = (idx + 1) % N;
        if (count < N) count++;
    }

    float average() const {
        if (count == 0) return 0.0f;
        float sum = 0.0f;
        for (size_t i = 0; i < count; i++) sum += buf[i];
        return sum / static_cast<float>(count);
    }

    void reset() {
        for (size_t i = 0; i < N; i++) buf[i] = 0.0f;
        idx = 0;
        count = 0;
    }
};

class MotorControl {
   public:
    struct Command {
        float left_wheel_angular_velocity = 0.0f;
        float right_wheel_angular_velocity = 0.0f;
        uint32_t seq = 0;
        uint32_t timeout_ms = 0;
        uint32_t last_update_ms = 0;
        bool active = false;
    };

    struct WheelState {
        int64_t ticks        = 0;     // absolute accumulated ticks (signed)
        float   velocity_mps = 0.0f;  // filtered velocity in m/s (signed)
    };

    void setup();
    static void task(void *arg);
    void setWheelCommand(
        float left_wheel_angular_velocity,
        float right_wheel_angular_velocity,
        uint32_t seq,
        uint32_t timeout_ms);
    void stop();
    bool expireIfTimedOut(uint32_t now_ms, uint32_t *expired_seq);
    Command getCommand() const;
    WheelState getLeftWheel() const { return left_wheel_; }
    WheelState getRightWheel() const { return right_wheel_; }

   private:
    // PI controller state
    struct PiState {
        float integral = 0.0f;
    };

    void setupEncoders();
    void readEncoders(float dt);
    float piControl(float setpoint, float measured, PiState &pi);
    void applyWheelDuties(float left_duty, float right_duty);

    // Command state
    volatile float left_wheel_angular_velocity_ = 0.0f;
    volatile float right_wheel_angular_velocity_ = 0.0f;
    volatile uint32_t seq_ = 0;
    volatile uint32_t timeout_ms_ = 0;
    volatile uint32_t last_update_ms_ = 0;
    volatile bool active_ = false;

    // Last commanded direction per wheel (for sign inference)
    float last_left_dir_  = 0.0f;
    float last_right_dir_ = 0.0f;

    // PCNT encoder units
    static constexpr pcnt_unit_t LEFT_PCNT_UNIT  = PCNT_UNIT_0;
    static constexpr pcnt_unit_t RIGHT_PCNT_UNIT = PCNT_UNIT_1;

    // Wheel state
    WheelState left_wheel_;
    WheelState right_wheel_;

    // Velocity moving average filters
    MovingAverage<VEL_AVG_WINDOW> left_vel_avg_;
    MovingAverage<VEL_AVG_WINDOW> right_vel_avg_;

    PiState left_pi_;
    PiState right_pi_;
};

#endif  // MOTOR_CONTROL_TASK_H
