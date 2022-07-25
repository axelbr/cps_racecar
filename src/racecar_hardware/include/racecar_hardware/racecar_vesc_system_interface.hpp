#ifndef RACECAR_HARDWARE__RACECAR_VESC_INTERFACE_HPP_
#define RACECAR_HARDWARE__RACECAR_VESC_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace racecar_hardware
{

struct FirmwareVersion {
  int major;
  int minor;
};


class VescSystemInterface: public hardware_interface::SystemInterface {
  
  public:
  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<vesc_driver::VescPacketImu const> imu_data_;
  std::shared_ptr<vesc_driver::VescPacketFWVersion const> firmware_data_;
  std::shared_ptr<vesc_driver::VescPacketValues const> state_;

  std::string hw_port_;
  vesc_driver::VescInterface vesc_;
  
  void packet_callback(const std::shared_ptr<vesc_driver::VescPacket const>& packet);
  void error_callback(const std::string& error);
};
    
} // namespace racecar_hardware


#endif