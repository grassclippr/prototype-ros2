#pragma once

#include <chrono>
#include <mutex>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rover_baseboard_msgs/msg/encoder_state.hpp>
#include <rover_baseboard_msgs/msg/motor_command.hpp>
#include <rover_baseboard_msgs/msg/safety_state.hpp>

namespace rover_hardware {

class RoverBaseboardSystem : public hardware_interface::SystemInterface {
   public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

   private:
    void encoderStateCallback(const rover_baseboard_msgs::msg::EncoderState::SharedPtr msg);
    void safetyStateCallback(const rover_baseboard_msgs::msg::SafetyState::SharedPtr msg);
    void publishWheelCommand();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rover_baseboard_msgs::msg::MotorCommand>::SharedPtr wheel_command_pub_;
    rclcpp::Subscription<rover_baseboard_msgs::msg::EncoderState>::SharedPtr encoder_state_sub_;
    rclcpp::Subscription<rover_baseboard_msgs::msg::SafetyState>::SharedPtr safety_state_sub_;
    std::mutex feedback_mutex_;
    std::vector<double> wheel_positions_;
    std::vector<double> wheel_velocities_;
    std::vector<double> wheel_commands_;
    std::vector<double> feedback_wheel_velocities_;
    std::chrono::steady_clock::time_point last_feedback_time_;
    double wheel_radius_ = 0.127;
    double ticks_per_meter_ = 150.0;
    double max_ticks_per_second_ = 30.0;
    double feedback_timeout_sec_ = 0.25;
    bool have_feedback_ = false;
    bool safety_interlock_triggered_ = false;
};

}  // namespace rover_hardware
