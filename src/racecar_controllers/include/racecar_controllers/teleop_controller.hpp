#ifndef RACECAR_CONTROLLERS__TELEOP_CONTROLLER_HPP_
#define RACECAR_CONTROLLERS__TELEOP_CONTROLLER_HPP_

#include <string>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>

namespace racecar_controllers
{

class TeleopController : public controller_interface::ControllerInterface
{
public:
  TeleopController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
 
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr command_sub;
    double effort = 0.0;
    double steering_angle = 0.0;
    std::string drive_train_command;
    double min_motor_command, max_motor_command, motor_command_scale, motor_command_offset;
};

}  // namespace teleop_controllers

#endif  // TELEOP_CONTROLLERS__JOINT_GROUP_TELEOP_CONTROLLER_HPP_