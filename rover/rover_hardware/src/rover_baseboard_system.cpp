#include "rover_hardware/rover_baseboard_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_hardware {

namespace {

constexpr size_t kExpectedJointCount = 2;
constexpr char kMotorCommandTopic[] = "/baseboard/motor_command";
constexpr char kEncoderStateTopic[] = "/baseboard/encoder_state";
constexpr char kSafetyStateTopic[] = "/baseboard/safety_state";
constexpr double kDefaultWheelRadiusMeters = 0.127;
constexpr double kDefaultTicksPerMeter = 72.0;
constexpr double kDefaultMaxTicksPerSecond = 40.0;
constexpr double kDefaultFeedbackTimeoutSec = 0.25;

bool loadPositiveDoubleParameter(
    const hardware_interface::HardwareInfo &info,
    const char *name,
    double default_value,
    double &value) {
    auto parameter_it = info.hardware_parameters.find(name);
    if (parameter_it == info.hardware_parameters.end()) {
        value = default_value;
        return true;
    }

    try {
        value = std::stod(parameter_it->second);
    } catch (const std::exception &) {
        return false;
    }

    return value > 0.0 && std::isfinite(value);
}

}  // namespace

hardware_interface::CallbackReturn RoverBaseboardSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != kExpectedJointCount) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Expected %zu joints, got %zu",
                     kExpectedJointCount,
                     info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    wheel_positions_.assign(info_.joints.size(), 0.0);
    wheel_velocities_.assign(info_.joints.size(), 0.0);
    wheel_commands_.assign(info_.joints.size(), 0.0);
    feedback_wheel_velocities_.assign(info_.joints.size(), 0.0);

    if (!loadPositiveDoubleParameter(info_, "wheel_radius", kDefaultWheelRadiusMeters, wheel_radius_)) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Invalid wheel_radius: %.6f",
                     wheel_radius_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!loadPositiveDoubleParameter(info_, "ticks_per_meter", kDefaultTicksPerMeter, ticks_per_meter_)) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Invalid ticks_per_meter: %.6f",
                     ticks_per_meter_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!loadPositiveDoubleParameter(
            info_, "max_ticks_per_second", kDefaultMaxTicksPerSecond, max_ticks_per_second_)) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Invalid max_ticks_per_second: %.6f",
                     max_ticks_per_second_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!loadPositiveDoubleParameter(
            info_, "feedback_timeout_sec", kDefaultFeedbackTimeoutSec, feedback_timeout_sec_)) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Invalid feedback_timeout_sec: %.6f",
                     feedback_timeout_sec_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = std::make_shared<rclcpp::Node>("rover_baseboard_system");
    wheel_command_pub_ = node_->create_publisher<rover_baseboard_msgs::msg::MotorCommand>(
        kMotorCommandTopic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    encoder_state_sub_ = node_->create_subscription<rover_baseboard_msgs::msg::EncoderState>(
        kEncoderStateTopic,
        rclcpp::SensorDataQoS(),
        [this](rover_baseboard_msgs::msg::EncoderState::SharedPtr msg) {
            encoderStateCallback(std::move(msg));
        });
    safety_state_sub_ = node_->create_subscription<rover_baseboard_msgs::msg::SafetyState>(
        kSafetyStateTopic,
        rclcpp::SensorDataQoS(),
        [this](rover_baseboard_msgs::msg::SafetyState::SharedPtr msg) {
            safetyStateCallback(std::move(msg));
        });

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverBaseboardSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(info_.joints.size() * 2);

    for (size_t index = 0; index < info_.joints.size(); ++index) {
        state_interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_POSITION, &wheel_positions_[index]);
        state_interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[index]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverBaseboardSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(info_.joints.size());

    for (size_t index = 0; index < info_.joints.size(); ++index) {
        command_interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &wheel_commands_[index]);
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn RoverBaseboardSystem::on_activate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    std::fill(wheel_positions_.begin(), wheel_positions_.end(), 0.0);
    std::fill(wheel_velocities_.begin(), wheel_velocities_.end(), 0.0);
    std::fill(wheel_commands_.begin(), wheel_commands_.end(), 0.0);
    std::fill(feedback_wheel_velocities_.begin(), feedback_wheel_velocities_.end(), 0.0);
    have_feedback_ = false;
    publishWheelCommand();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverBaseboardSystem::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;
    std::fill(wheel_commands_.begin(), wheel_commands_.end(), 0.0);
    publishWheelCommand();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverBaseboardSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;

    if (node_) {
        rclcpp::spin_some(node_);
    }

    const double dt_seconds = period.seconds();
    const auto now = std::chrono::steady_clock::now();
    std::vector<double> measured_velocities;
    bool feedback_is_fresh = false;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        measured_velocities = feedback_wheel_velocities_;
        feedback_is_fresh = have_feedback_ &&
                             std::chrono::duration<double>(now - last_feedback_time_).count() <=
                                 feedback_timeout_sec_;
    }

    for (size_t index = 0; index < wheel_positions_.size(); ++index) {
        wheel_velocities_[index] = feedback_is_fresh ? measured_velocities[index] : 0.0;
        if (feedback_is_fresh) {
            wheel_positions_[index] += wheel_velocities_[index] * dt_seconds;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverBaseboardSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;
    (void)period;
    publishWheelCommand();
    return hardware_interface::return_type::OK;
}

void RoverBaseboardSystem::encoderStateCallback(const rover_baseboard_msgs::msg::EncoderState::SharedPtr msg) {
    const double left_linear_mps = msg->left_velocity_ticks_per_sec / ticks_per_meter_;
    const double right_linear_mps = msg->right_velocity_ticks_per_sec / ticks_per_meter_;
    if (!std::isfinite(left_linear_mps) || !std::isfinite(right_linear_mps)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(),
                             *node_->get_clock(),
                             1000,
                             "Ignoring non-finite wheel feedback: left=%.6f right=%.6f",
                             left_linear_mps,
                             right_linear_mps);
        return;
    }

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!feedback_wheel_velocities_.empty()) {
        feedback_wheel_velocities_[0] = left_linear_mps / wheel_radius_;
    }
    if (feedback_wheel_velocities_.size() > 1) {
        feedback_wheel_velocities_[1] = right_linear_mps / wheel_radius_;
    }
    last_feedback_time_ = std::chrono::steady_clock::now();
    have_feedback_ = true;
}

void RoverBaseboardSystem::safetyStateCallback(const rover_baseboard_msgs::msg::SafetyState::SharedPtr msg) {
    if (msg->interlock_triggered && !safety_interlock_triggered_) {
        RCLCPP_WARN(node_->get_logger(), "Baseboard safety interlock triggered");
    } else if (!msg->interlock_triggered && safety_interlock_triggered_) {
        RCLCPP_INFO(node_->get_logger(), "Baseboard safety interlock released");
    }
    safety_interlock_triggered_ = msg->interlock_triggered;
}

void RoverBaseboardSystem::publishWheelCommand() {
    if (!wheel_command_pub_) {
        return;
    }

    const auto to_percent = [this](double joint_rad_per_sec) {
        if (!std::isfinite(joint_rad_per_sec)) {
            return 0.0;
        }
        const double linear_mps = joint_rad_per_sec * wheel_radius_;
        const double ticks_per_second = linear_mps * ticks_per_meter_;
        return std::clamp(100.0 * ticks_per_second / max_ticks_per_second_, -100.0, 100.0);
    };

    rover_baseboard_msgs::msg::MotorCommand msg;
    if (!wheel_commands_.empty()) {
        msg.left_speed_percent = to_percent(wheel_commands_[0]);
    }
    if (wheel_commands_.size() > 1) {
        msg.right_speed_percent = to_percent(wheel_commands_[1]);
    }
    msg.timeout_ms = 500;
    wheel_command_pub_->publish(msg);
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(rover_hardware::RoverBaseboardSystem, hardware_interface::SystemInterface)
