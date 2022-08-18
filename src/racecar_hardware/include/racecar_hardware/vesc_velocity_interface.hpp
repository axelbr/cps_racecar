#ifndef RACECAR_HARDWARE__RACECAR_VESC_VELOCITY_INTERFACE_HPP_
#define RACECAR_HARDWARE__RACECAR_VESC_VELOCITY_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/actuator_interface.hpp"
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

struct FirmwareVersion
{
  int major;
  int minor;
};

struct State
{
  double velocity;
  double steering_angle;
};

class VescVelocityInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::shared_ptr<vesc_driver::VescPacketImu const> imu_data_;
  std::shared_ptr<vesc_driver::VescPacketFWVersion const> firmware_data_;
  std::shared_ptr<vesc_driver::VescPacketValues const> vesc_values_;

  std::string hw_port_;
  double erpm_gain_, erpm_offset_, servo_gain_, servo_offset_;
  vesc_driver::VescInterface vesc_;

  double target_velocity_, target_steering_angle_;
  racecar_hardware::State state_;

  void packet_callback(const std::shared_ptr<vesc_driver::VescPacket const>& packet);
  void error_callback(const std::string& error);
  bool check_servo_joint(const hardware_interface::ComponentInfo& servo_joint);
  bool check_motor_joint(const hardware_interface::ComponentInfo& motor_joint);
};

}  // namespace racecar_hardware

#endif