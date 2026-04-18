#include "rover_hardware/rover_baseboard_system.hpp"

#include <algorithm>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rover_hardware {

namespace {

constexpr size_t kExpectedJointCount = 2;

}  // namespace

hardware_interface::CallbackReturn RoverBaseboardSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &info = params.hardware_info;

    if (info_.joints.size() != kExpectedJointCount) {
        RCLCPP_ERROR(rclcpp::get_logger("RoverBaseboardSystem"),
                     "Expected %zu joints, got %zu",
                     kExpectedJointCount,
                     info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto topic_it = info_.hardware_parameters.find("wheel_command_topic");
    if (topic_it != info_.hardware_parameters.end() && !topic_it->second.empty()) {
        wheel_command_topic_ = topic_it->second;
    }

    wheel_positions_.assign(info_.joints.size(), 0.0);
    wheel_velocities_.assign(info_.joints.size(), 0.0);
    wheel_commands_.assign(info_.joints.size(), 0.0);

    node_ = std::make_shared<rclcpp::Node>("rover_baseboard_system");
    wheel_command_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(wheel_command_topic_, 10);

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

    const double dt_seconds = period.seconds();
    for (size_t index = 0; index < wheel_positions_.size(); ++index) {
        wheel_velocities_[index] = wheel_commands_[index];
        wheel_positions_[index] += wheel_velocities_[index] * dt_seconds;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverBaseboardSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    (void)time;
    (void)period;
    publishWheelCommand();
    return hardware_interface::return_type::OK;
}

void RoverBaseboardSystem::publishWheelCommand() {
    if (!wheel_command_pub_) {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(wheel_commands_.size());
    for (const double command : wheel_commands_) {
        msg.data.push_back(static_cast<float>(command));
    }
    wheel_command_pub_->publish(msg);
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(rover_hardware::RoverBaseboardSystem, hardware_interface::SystemInterface)
