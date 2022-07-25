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
  hardware_interface::CallbackReturn VescSystemInterface::on_init(const hardware_interface::HardwareInfo & info) 
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_port_ = info_.hardware_parameters["port"];
    try {
      vesc_.connect(hw_port_);
      vesc_.setPacketHandler([this](const auto& packet) {
        this->packet_callback(packet);
      });
      vesc_.setErrorHandler([this](const auto& error) {
        this->error_callback(error);
      });
    } catch (std::exception& e) {
        RCLCPP_FATAL(
            rclcpp::get_logger("VescSystemInterface"),
            "Failed to connect to the VESC, %s.", e.what()
        );
        return hardware_interface::CallbackReturn::ERROR; 
    }

    vesc_.requestFWVersion();
    return hardware_interface::CallbackReturn::SUCCESS;
}

void VescSystemInterface::packet_callback(const std::shared_ptr<vesc_driver::VescPacket const>& packet) 
{
  if (packet->name() == "Values") {
    state_ = std::dynamic_pointer_cast<vesc_driver::VescPacketValues const>(packet);
    
  }
  RCLCPP_INFO(rclcpp::get_logger("VescSystemInterface"), "Received Packet from VESC: %s", packet->name().c_str());
}

std::vector<hardware_interface::StateInterface> VescSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  return command_interfaces;
}

hardware_interface::CallbackReturn VescSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type racecar_hardware::VescSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace racecar_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_hardware::VescSystemInterface, hardware_interface::SystemInterface)