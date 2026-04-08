#include "./task.h"

#include <Arduino.h>
#include <cmath>

#include "tasks/uros/serial_mux_debug.h"

#define printf serial_mux::debug_printf

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr gpio_num_t RIGHT_ENABLE_PIN = GPIO_NUM_8;
constexpr gpio_num_t RIGHT_FORWARD_PIN = GPIO_NUM_9;
constexpr gpio_num_t RIGHT_REVERSE_PIN = GPIO_NUM_10;
constexpr gpio_num_t LEFT_ENABLE_PIN = GPIO_NUM_11;
constexpr gpio_num_t LEFT_FORWARD_PIN = GPIO_NUM_12;
constexpr gpio_num_t LEFT_REVERSE_PIN = GPIO_NUM_13;

constexpr ledc_mode_t PWM_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQUENCY_HZ = 2000;
constexpr uint32_t PWM_MAX_DUTY = (1U << 10U) - 1U;

constexpr ledc_channel_t LEFT_FORWARD_CHANNEL = LEDC_CHANNEL_0;
constexpr ledc_channel_t LEFT_REVERSE_CHANNEL = LEDC_CHANNEL_1;
constexpr ledc_channel_t RIGHT_FORWARD_CHANNEL = LEDC_CHANNEL_2;
constexpr ledc_channel_t RIGHT_REVERSE_CHANNEL = LEDC_CHANNEL_3;

constexpr bool MOTOR_ENABLE_ACTIVE_HIGH = true;
// Chassis forward should map to positive wheel commands in the rover frame.
// If the rover drives backward for a positive forward command, flip these.
constexpr bool LEFT_MOTOR_INVERTED = true;
constexpr bool RIGHT_MOTOR_INVERTED = false;

// Keep the initial bring-up conservative.
constexpr float TRACK_WIDTH_METERS = 0.66f;
constexpr float MAX_WHEEL_LINEAR_SPEED_MPS = 0.20f;
constexpr float COMMAND_DEADBAND_MPS = 0.01f;

void setEnablePin(gpio_num_t pin, bool enabled) {
    const int level = (enabled == MOTOR_ENABLE_ACTIVE_HIGH) ? 1 : 0;
    gpio_set_level(pin, level);
}

void writeDuty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

uint32_t dutyForSpeed(float speed_mps) {
    const float normalized = std::fmin(std::fabs(speed_mps) / MAX_WHEEL_LINEAR_SPEED_MPS, 1.0f);
    return static_cast<uint32_t>(normalized * static_cast<float>(PWM_MAX_DUTY));
}

float applyMotorPolarity(float wheel_speed, bool inverted) {
    return inverted ? -wheel_speed : wheel_speed;
}

void configurePwmChannel(ledc_channel_t channel, gpio_num_t pin) {
    ledc_channel_config_t config = {};
    config.gpio_num = pin;
    config.speed_mode = PWM_MODE;
    config.channel = channel;
    config.intr_type = LEDC_INTR_DISABLE;
    config.timer_sel = PWM_TIMER;
    config.duty = 0;
    config.hpoint = 0;
    ledc_channel_config(&config);
}
}  // namespace

void MotorControl::setup() {
    printf("motor setup: pwm=%luHz track=%.2fm inverted(L,R)=(%d,%d)\n",
           static_cast<unsigned long>(PWM_FREQUENCY_HZ),
           static_cast<double>(TRACK_WIDTH_METERS),
           LEFT_MOTOR_INVERTED ? 1 : 0,
           RIGHT_MOTOR_INVERTED ? 1 : 0);

    gpio_config_t enable_pin_config = {};
    enable_pin_config.pin_bit_mask = (1ULL << RIGHT_ENABLE_PIN) | (1ULL << LEFT_ENABLE_PIN);
    enable_pin_config.mode = GPIO_MODE_OUTPUT;
    enable_pin_config.pull_up_en = GPIO_PULLUP_DISABLE;
    enable_pin_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enable_pin_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&enable_pin_config);

    setEnablePin(LEFT_ENABLE_PIN, false);
    setEnablePin(RIGHT_ENABLE_PIN, false);

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = PWM_MODE;
    timer_config.timer_num = PWM_TIMER;
    timer_config.duty_resolution = PWM_RESOLUTION;
    timer_config.freq_hz = PWM_FREQUENCY_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_config);

    configurePwmChannel(LEFT_FORWARD_CHANNEL, LEFT_FORWARD_PIN);
    configurePwmChannel(LEFT_REVERSE_CHANNEL, LEFT_REVERSE_PIN);
    configurePwmChannel(RIGHT_FORWARD_CHANNEL, RIGHT_FORWARD_PIN);
    configurePwmChannel(RIGHT_REVERSE_CHANNEL, RIGHT_REVERSE_PIN);

    applyWheelOutputs(0.0f, 0.0f);

    xTaskCreate(
        task,
        "motorTask",
        4096,
        this,
        1,
        NULL);
}

void MotorControl::task(void *arg) {
    MotorControl *self = static_cast<MotorControl *>(arg);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&xLastWakeTime, 10 / portTICK_RATE_MS);

        const Command command = self->getCommand();
        if (!command.active) {
            self->applyWheelOutputs(0.0f, 0.0f);
            continue;
        }

        const float half_track = TRACK_WIDTH_METERS * 0.5f;
        float left_speed = command.linear_x - (command.angular_z * half_track);
        float right_speed = command.linear_x + (command.angular_z * half_track);

        if (std::fabs(left_speed) < COMMAND_DEADBAND_MPS) {
            left_speed = 0.0f;
        }
        if (std::fabs(right_speed) < COMMAND_DEADBAND_MPS) {
            right_speed = 0.0f;
        }

        self->applyWheelOutputs(left_speed, right_speed);
    }
}

void MotorControl::applyWheelOutputs(float left_speed, float right_speed) {
    left_speed = applyMotorPolarity(left_speed, LEFT_MOTOR_INVERTED);
    right_speed = applyMotorPolarity(right_speed, RIGHT_MOTOR_INVERTED);

    const uint32_t left_duty = dutyForSpeed(left_speed);
    const uint32_t right_duty = dutyForSpeed(right_speed);

    printf("motor outputs: left=%.3f m/s right=%.3f m/s duty(L,R)=(%lu,%lu)\n",
           static_cast<double>(left_speed),
           static_cast<double>(right_speed),
           static_cast<unsigned long>(left_duty),
           static_cast<unsigned long>(right_duty));

    writeDuty(LEFT_FORWARD_CHANNEL, left_speed > 0.0f ? left_duty : 0U);
    writeDuty(LEFT_REVERSE_CHANNEL, left_speed < 0.0f ? left_duty : 0U);
    writeDuty(RIGHT_FORWARD_CHANNEL, right_speed > 0.0f ? right_duty : 0U);
    writeDuty(RIGHT_REVERSE_CHANNEL, right_speed < 0.0f ? right_duty : 0U);

    setEnablePin(LEFT_ENABLE_PIN, left_duty > 0U);
    setEnablePin(RIGHT_ENABLE_PIN, right_duty > 0U);
}

void MotorControl::setCommand(float linear_x, float angular_z, uint32_t seq, uint32_t timeout_ms) {
    printf("motor command: linear_x=%.3f angular_z=%.3f seq=%lu timeout_ms=%lu\n",
           static_cast<double>(linear_x),
           static_cast<double>(angular_z),
           static_cast<unsigned long>(seq),
           static_cast<unsigned long>(timeout_ms));

    linear_x_ = linear_x;
    angular_z_ = angular_z;
    seq_ = seq;
    timeout_ms_ = timeout_ms;
    last_update_ms_ = millis();
    active_ = true;
    applyWheelOutputs(linear_x_, angular_z_);
}

void MotorControl::stop() {
    printf("motor stop\n");
    linear_x_ = 0.0f;
    angular_z_ = 0.0f;
    active_ = false;
    applyWheelOutputs(0.0f, 0.0f);
}

bool MotorControl::expireIfTimedOut(uint32_t now_ms, uint32_t *expired_seq) {
    if (!active_ || timeout_ms_ == 0) {
        return false;
    }

    if (static_cast<uint32_t>(now_ms - last_update_ms_) < timeout_ms_) {
        return false;
    }

    if (expired_seq != nullptr) {
        *expired_seq = seq_;
    }
    printf("motor command timed out: seq=%lu timeout_ms=%lu\n",
           static_cast<unsigned long>(seq_),
           static_cast<unsigned long>(timeout_ms_));
    stop();
    return true;
}

MotorControl::Command MotorControl::getCommand() const {
    Command command;
    command.linear_x = linear_x_;
    command.angular_z = angular_z_;
    command.seq = seq_;
    command.timeout_ms = timeout_ms_;
    command.last_update_ms = last_update_ms_;
    command.active = active_;
    return command;
}
