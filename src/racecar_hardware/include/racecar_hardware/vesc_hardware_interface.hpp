#ifndef RACECAR_HARDWARE__RACECAR_VESC_HARDWARE_INTERFACE_HPP_
#define RACECAR_HARDWARE__RACECAR_VESC_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

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

struct VescPacketData
{
  std::shared_ptr<vesc_driver::VescPacketImu const> imu_data;
  std::shared_ptr<vesc_driver::VescPacketFWVersion const> firmware_data;
  std::shared_ptr<vesc_driver::VescPacketValues const> motor_data;
};

struct VescConfig {
  std::string hw_port;
  double servo_gain, servo_offset, transmission_ratio;
  int wheel_diameter, motor_poles;
  bool activate_duty_cycle, activate_current, activate_erpm;
};

struct ControlInterface {
  public:
    double min, max, value;
    double clamped_value() const {
      return std::clamp(value, this->min, this->max);
    }
  private:
      
};

enum MotorCommandType {
  CURRENT,
  DUTY_CYCLE,
  ERPM
};

class VescHardwareInterface : public hardware_interface::SystemInterface
{

public:

  typedef std::map<std::string, std::map<std::string, ControlInterface>> VescControls;
  typedef std::map<std::string, std::map<std::string, double>> VescStates;


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
  std::shared_ptr<vesc_driver::VescPacketValues const> motor_data_;

  VescConfig config_;
  VescControls controls_;
  VescStates states_;
  VescPacketData packet_data_;
  vesc_driver::VescInterface vesc_;
  MotorCommandType command_type_;
  std::unordered_map<std::string, MotorCommandType> command_types = {
    {"current", MotorCommandType::CURRENT},
    {"duty_cycle", MotorCommandType::DUTY_CYCLE},
    {"erpm", MotorCommandType::ERPM}
  };

  void packet_callback(const std::shared_ptr<vesc_driver::VescPacket const>& packet);
  void error_callback(const std::string& error);
  void set_interface_fields(const hardware_interface::HardwareInfo& info);
  bool check_motor_info(const hardware_interface::ComponentInfo& info);
  bool check_steering_info(const hardware_interface::ComponentInfo& info);
};

}  // namespace racecar_hardware

#endif