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
constexpr char kWheelCommandTopic[] = "/wheel_cmd";
constexpr char kWheelVelocityTopic[] = "/wheel_velocities";
constexpr double kDefaultWheelRadiusMeters = 0.127;
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

    if (!loadPositiveDoubleParameter(
            info_, "feedback_timeout_sec", kDefaultFeedbackTimeoutSec, feedback_timeout_sec_)) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Invalid feedback_timeout_sec: %.6f",
                     feedback_timeout_sec_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = std::make_shared<rclcpp::Node>("rover_baseboard_system");
    wheel_command_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        kWheelCommandTopic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    wheel_velocity_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        kWheelVelocityTopic,
        rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
            wheelVelocityCallback(std::move(msg));
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

void RoverBaseboardSystem::wheelVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double left_linear_mps = msg->linear.y;
    const double right_linear_mps = msg->linear.z;
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

void RoverBaseboardSystem::publishWheelCommand() {
    if (!wheel_command_pub_) {
        return;
    }

    geometry_msgs::msg::Twist msg;
    if (!wheel_commands_.empty()) {
        msg.linear.x = wheel_commands_[0];
    }
    if (wheel_commands_.size() > 1) {
        msg.linear.y = wheel_commands_[1];
    }
    wheel_command_pub_->publish(msg);
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(rover_hardware::RoverBaseboardSystem, hardware_interface::SystemInterface)
