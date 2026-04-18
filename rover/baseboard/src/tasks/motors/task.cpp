#include "./task.h"

#include <Arduino.h>
#include <cmath>

#include "driver/gpio.h"
#include "hardware.h"
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
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = true;

constexpr float WHEEL_RADIUS_METERS = 0.127f;
constexpr float COMMAND_DEADBAND_RADPS = 0.01f;

// Encoder pins (single-channel, asymmetric — count rising edges only)
constexpr gpio_num_t LEFT_ENCODER_PIN  = static_cast<gpio_num_t>(I2C_SDA_PIN);
constexpr gpio_num_t RIGHT_ENCODER_PIN = static_cast<gpio_num_t>(I2C_SCL_PIN);

void setEnablePin(gpio_num_t pin, bool enabled) {
    const int level = (enabled == MOTOR_ENABLE_ACTIVE_HIGH) ? 1 : 0;
    gpio_set_level(pin, level);
}

void writeDuty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
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

void initPcntUnit(pcnt_unit_t unit, gpio_num_t pin) {
    pcnt_config_t config = {};
    config.pulse_gpio_num = pin;
    config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    config.lctrl_mode     = PCNT_MODE_KEEP;
    config.hctrl_mode     = PCNT_MODE_KEEP;
    config.pos_mode       = PCNT_COUNT_INC;  // count on rising edge
    config.neg_mode       = PCNT_COUNT_DIS;  // ignore falling edge (asymmetric)
    config.counter_h_lim  = 32767;
    config.counter_l_lim  = -1;
    config.unit           = unit;
    config.channel        = PCNT_CHANNEL_0;
    ESP_ERROR_CHECK(pcnt_unit_config(&config));

    // Optional glitch filter — ignore pulses shorter than ~1 us (80 APB clocks)
    ESP_ERROR_CHECK(pcnt_set_filter_value(unit, 80));
    ESP_ERROR_CHECK(pcnt_filter_enable(unit));

    ESP_ERROR_CHECK(pcnt_counter_pause(unit));
    ESP_ERROR_CHECK(pcnt_counter_clear(unit));
    ESP_ERROR_CHECK(pcnt_counter_resume(unit));
}
}  // namespace

void MotorControl::setupEncoders() {
    initPcntUnit(LEFT_PCNT_UNIT, LEFT_ENCODER_PIN);
    initPcntUnit(RIGHT_PCNT_UNIT, RIGHT_ENCODER_PIN);
    printf("PCNT encoders initialized (left=GPIO%d, right=GPIO%d)\n",
           LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);
}

void MotorControl::readEncoders(float dt) {
    int16_t left_count = 0, right_count = 0;
    pcnt_get_counter_value(LEFT_PCNT_UNIT, &left_count);
    pcnt_counter_clear(LEFT_PCNT_UNIT);
    pcnt_get_counter_value(RIGHT_PCNT_UNIT, &right_count);
    pcnt_counter_clear(RIGHT_PCNT_UNIT);

    // Infer sign from last commanded PWM direction
    float left_sign  = (last_left_dir_ >= 0.0f) ? 1.0f : -1.0f;
    float right_sign = (last_right_dir_ >= 0.0f) ? 1.0f : -1.0f;

    int left_signed  = static_cast<int>(left_sign)  * left_count;
    int right_signed = static_cast<int>(right_sign) * right_count;

    // Accumulate absolute ticks
    left_wheel_.ticks  += left_signed;
    right_wheel_.ticks += right_signed;

    // Compute raw velocity and push into moving average
    float left_raw_vel  = (static_cast<float>(left_signed)  / TICKS_PER_METER) / dt;
    float right_raw_vel = (static_cast<float>(right_signed) / TICKS_PER_METER) / dt;

    left_vel_avg_.push(left_raw_vel);
    right_vel_avg_.push(right_raw_vel);

    left_wheel_.velocity_mps  = left_vel_avg_.average();
    right_wheel_.velocity_mps = right_vel_avg_.average();
}

float MotorControl::piControl(float setpoint, float measured, PiState &pi) {
    float error = setpoint - measured;

    pi.integral += error * CONTROL_PERIOD_S;
    // Anti-windup clamp
    if (pi.integral > PI_INTEGRAL_LIMIT) pi.integral = PI_INTEGRAL_LIMIT;
    if (pi.integral < -PI_INTEGRAL_LIMIT) pi.integral = -PI_INTEGRAL_LIMIT;

    float output = PI_KP * error + PI_KI * pi.integral;

    // Clamp output to [-1, 1]
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;

    return output;
}

void MotorControl::setup() {
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

    applyWheelDuties(0.0f, 0.0f);

    setupEncoders();

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

        // Always read encoders to keep position tracking up to date
        self->readEncoders(CONTROL_PERIOD_S);

        const Command command = self->getCommand();
        if (!command.active) {
            self->left_pi_.integral  = 0.0f;
            self->right_pi_.integral = 0.0f;
            self->applyWheelDuties(0.0f, 0.0f);
            continue;
        }

        float left_wheel_angular_velocity = command.left_wheel_angular_velocity;
        float right_wheel_angular_velocity = command.right_wheel_angular_velocity;

        if (std::fabs(left_wheel_angular_velocity) < COMMAND_DEADBAND_RADPS) {
            left_wheel_angular_velocity = 0.0f;
        }
        if (std::fabs(right_wheel_angular_velocity) < COMMAND_DEADBAND_RADPS) {
            right_wheel_angular_velocity = 0.0f;
        }

        const float left_setpoint = left_wheel_angular_velocity * WHEEL_RADIUS_METERS;
        const float right_setpoint = right_wheel_angular_velocity * WHEEL_RADIUS_METERS;

        // Remember direction for encoder sign inference
        self->last_left_dir_  = left_setpoint;
        self->last_right_dir_ = right_setpoint;

        float left_duty  = self->piControl(left_setpoint,  self->left_wheel_.velocity_mps,  self->left_pi_);
        float right_duty = self->piControl(right_setpoint, self->right_wheel_.velocity_mps, self->right_pi_);

        self->applyWheelDuties(left_duty, right_duty);
    }
}

void MotorControl::applyWheelDuties(float left_duty, float right_duty) {
    // Apply motor polarity inversion
    if (LEFT_MOTOR_INVERTED)  left_duty  = -left_duty;
    if (RIGHT_MOTOR_INVERTED) right_duty = -right_duty;

    uint32_t left_abs  = static_cast<uint32_t>(std::fmin(std::fabs(left_duty), 1.0f) * PWM_MAX_DUTY);
    uint32_t right_abs = static_cast<uint32_t>(std::fmin(std::fabs(right_duty), 1.0f) * PWM_MAX_DUTY);

    writeDuty(LEFT_FORWARD_CHANNEL,  left_duty > 0.0f  ? left_abs  : 0U);
    writeDuty(LEFT_REVERSE_CHANNEL,  left_duty < 0.0f  ? left_abs  : 0U);
    writeDuty(RIGHT_FORWARD_CHANNEL, right_duty > 0.0f ? right_abs : 0U);
    writeDuty(RIGHT_REVERSE_CHANNEL, right_duty < 0.0f ? right_abs : 0U);

    setEnablePin(LEFT_ENABLE_PIN,  left_abs > 0U);
    setEnablePin(RIGHT_ENABLE_PIN, right_abs > 0U);
}

void MotorControl::setWheelCommand(
    float left_wheel_angular_velocity,
    float right_wheel_angular_velocity,
    uint32_t seq,
    uint32_t timeout_ms) {
    printf("wheel command: left=%.3f rad/s right=%.3f rad/s seq=%lu timeout_ms=%lu\n",
           static_cast<double>(left_wheel_angular_velocity),
           static_cast<double>(right_wheel_angular_velocity),
           static_cast<unsigned long>(seq),
           static_cast<unsigned long>(timeout_ms));

    left_wheel_angular_velocity_ = left_wheel_angular_velocity;
    right_wheel_angular_velocity_ = right_wheel_angular_velocity;
    seq_ = seq;
    timeout_ms_ = timeout_ms;
    last_update_ms_ = millis();
    active_ = true;
    // Actual duty is computed by PI loop in task()
}

void MotorControl::stop() {
    left_wheel_angular_velocity_ = 0.0f;
    right_wheel_angular_velocity_ = 0.0f;
    active_ = false;
    left_pi_.integral  = 0.0f;
    right_pi_.integral = 0.0f;
    left_vel_avg_.reset();
    right_vel_avg_.reset();
    applyWheelDuties(0.0f, 0.0f);
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
    stop();
    return true;
}

MotorControl::Command MotorControl::getCommand() const {
    Command command;
    command.left_wheel_angular_velocity = left_wheel_angular_velocity_;
    command.right_wheel_angular_velocity = right_wheel_angular_velocity_;
    command.seq = seq_;
    command.timeout_ms = timeout_ms_;
    command.last_update_ms = last_update_ms_;
    command.active = active_;
    return command;
}
