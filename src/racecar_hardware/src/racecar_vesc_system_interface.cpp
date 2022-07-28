#include "racecar_hardware/racecar_vesc_system_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cassert>
#include <sstream>
#include <string>
#include <exception>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace racecar_hardware
{
hardware_interface::CallbackReturn VescInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_port_ = info_.hardware_parameters["vesc_port"];
  erpm_gain_ = std::stod(info_.hardware_parameters["erpm_gain"]);
  erpm_offset_ = std::stod(info_.hardware_parameters["erpm_offset"]);
  servo_gain_ = std::stod(info_.hardware_parameters["servo_gain"]);
  servo_offset_ = std::stod(info_.hardware_parameters["servo_offset"]);
  target_velocity_ = 0;
  target_steering_angle_ = 0;

  if (info_.joints.size() != 2 or info_.joints[0].name != "engine" or info_.joints[1].name != "servo")
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescInterface"), "Expected 2 joints: [engine, servo]");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto& motor = info_.joints[0];
  const auto& servo = info_.joints[1];

  if (not check_motor_joint(motor) or not check_servo_joint(servo))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool VescInterface::check_motor_joint(const hardware_interface::ComponentInfo& motor_joint)
{
  if (motor_joint.command_interfaces.size() != 1 or
      motor_joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescInterface"),
                 "Expected 1 command interface \"velocity\" for joint "
                 "\"engine\".");
    return false;
  }

  if (motor_joint.state_interfaces.size() != 1 or
      motor_joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescInterface"), "Expected 1 state interface \"velocity\" for joint \"engine\".");
    return false;
  }

  return true;
}

bool VescInterface::check_servo_joint(const hardware_interface::ComponentInfo& servo_joint)
{
  if (servo_joint.command_interfaces.size() != 1 or
      servo_joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescInterface"), "Expected 1 command interface \"position\" for joint \"servo\".");
    return false;
  }

  if (servo_joint.state_interfaces.size() != 1 or
      servo_joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescInterface"), "Expected 1 state interface \"position\" for joint \"servo\".");
    return false;
  }

  return true;
}

void VescInterface::packet_callback(const std::shared_ptr<vesc_driver::VescPacket const>& packet)
{
  if (packet->name() == "Values")
  {
    vesc_values_ = std::dynamic_pointer_cast<vesc_driver::VescPacketValues const>(packet);
  }

  if (packet->name() == "FWVersion")
  {
    firmware_data_ = std::dynamic_pointer_cast<vesc_driver::VescPacketFWVersion const>(packet);
  }
}

void VescInterface::error_callback(const std::string& error)
{
  RCLCPP_ERROR(rclcpp::get_logger("VescInterface"), "Received error from VESC: %s", error.c_str());
}

std::vector<hardware_interface::StateInterface> VescInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("engine", hardware_interface::HW_IF_VELOCITY, &state_.velocity));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface("servo", hardware_interface::HW_IF_POSITION, &state_.steering_angle));
  RCLCPP_INFO(rclcpp::get_logger("VescInterface"), "Exporting state interfaces.");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("engine", hardware_interface::HW_IF_VELOCITY, &target_velocity_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("servo", hardware_interface::HW_IF_POSITION, &target_steering_angle_));
  RCLCPP_INFO(rclcpp::get_logger("VescInterface"), "Exporting command interfaces.");
  return command_interfaces;
}

hardware_interface::CallbackReturn VescInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    vesc_.connect(hw_port_);
    vesc_.setPacketHandler([this](const auto& packet) { this->packet_callback(packet); });
    vesc_.setErrorHandler([this](const auto& error) { this->error_callback(error); });
    vesc_.requestFWVersion();
  }
  catch (std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VescInterface"), "Failed to connect to the VESC: %s.", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("VescInterface"), "Successfully connected to VESC.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  vesc_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  vesc_.requestState();
  if (not vesc_values_)
  {
    RCLCPP_WARN(rclcpp::get_logger("VescInterface"), "VESC packet was empty.");
    return hardware_interface::return_type::OK;
  }

  state_.velocity = (-vesc_values_->rpm() - erpm_offset_) / erpm_gain_;
  state_.steering_angle = target_steering_angle_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type racecar_hardware::VescInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  double rpm =  target_velocity_ * erpm_gain_ + erpm_offset_;
  double servo = target_steering_angle_ * servo_gain_ + servo_offset_;
  vesc_.setSpeed(rpm);
  vesc_.setServo(servo);
  return hardware_interface::return_type::OK;
}

}  // namespace racecar_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_hardware::VescInterface, hardware_interface::SystemInterface)