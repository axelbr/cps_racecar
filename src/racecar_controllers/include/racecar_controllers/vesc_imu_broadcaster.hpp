#ifndef RACECAR_CONTROLLERS__VESC_IMU_BROADCASTER_HPP_
#define RACECAR_CONTROLLERS__VESC_IMU_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "joint_state_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace racecar_controllers
{

class VescImuBroadcaster : public controller_interface::ControllerInterface
{
public:
  
  VescImuBroadcaster();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  using ImuPublisher = rclcpp::Publisher<sensor_msgs::msg::Imu>;
  ImuPublisher::SharedPtr  publisher_;
  std::unordered_map<std::string, double> states_;
};

} 

#endif  // RACECAR_CONTROLLERS__VESC_IMU_BROADCASTER_HPP_