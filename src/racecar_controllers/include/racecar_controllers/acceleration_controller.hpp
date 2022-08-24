#ifndef RACECAR_CONTROLLERS__TELEOP_CONTROLLER_HPP_
#define RACECAR_CONTROLLERS__TELEOP_CONTROLLER_HPP_

#include <string>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "pid/PID.hpp"

namespace racecar_controllers
{

  class AccelerationController : public controller_interface::ControllerInterface
  {
  public:
    AccelerationController();
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr command_sub;

    struct PIDParams {
      double P, I, D, min, max;
    };

    struct Parameters {
      PIDParams acceleration_controller_params, velocity_controller_params;
      std::string command_interface_name;
      std::string acceleration_sensor_name;
      std::string velocity_sensor_name;
    };

    struct MotorHandle
    {
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> acceleration;
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
      std::reference_wrapper<hardware_interface::LoanedCommandInterface> current;
    };

    std::unique_ptr<MotorHandle> motor_handle_;
    Parameters params_;
    std::unique_ptr<PIDController<double>> speed_controller_;
    std::unique_ptr<PIDController<double>> acceleration_controller_;
    double current = 0.0;
  };

} // namespace teleop_controllers

#endif // TELEOP_CONTROLLERS__JOINT_GROUP_TELEOP_CONTROLLER_HPP_