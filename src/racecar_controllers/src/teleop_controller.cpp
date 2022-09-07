#include "racecar_controllers/teleop_controller.hpp"
#include "rclcpp/logging.hpp"
#include <algorithm>

namespace racecar_controllers
{

  TeleopController::TeleopController() : controller_interface::ControllerInterface()
  {

  }

  controller_interface::CallbackReturn TeleopController::on_init()
  {
    auto_declare<std::string>("drive_train_command", std::string("current"));
    auto_declare<double>("min_motor_command", -10.0);
    auto_declare<double>("motor_command_scale", 10.0);
    auto_declare<double>("motor_command_offset", 0);
    auto_declare<double>("max_motor_command", 10.0);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn TeleopController::on_configure(const rclcpp_lifecycle::State &previous_state)
  { 
    drive_train_command = get_node()->get_parameter("drive_train_command").as_string();
    min_motor_command = get_node()->get_parameter("min_motor_command").as_double();
    max_motor_command = get_node()->get_parameter("max_motor_command").as_double();
    motor_command_scale = get_node()->get_parameter("motor_command_scale").as_double();
    motor_command_offset = get_node()->get_parameter("motor_command_offset").as_double();
    
    auto callback = [this](const sensor_msgs::msg::Joy &msg)
    {
      if (msg.buttons[4] == 1) {
        this->effort = std::clamp(motor_command_scale * msg.axes[1] + motor_command_offset, min_motor_command, max_motor_command);
        this->steering_angle = msg.axes[3];
      } else {
        this->effort = 0.0;
        this->steering_angle = 0.0;
      }
      RCLCPP_INFO(this->get_node()->get_logger(), "Effort=%f", msg.buttons[4], this->effort);
    };
    command_sub = this->get_node()->create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration TeleopController::command_interface_configuration() const
  {
    auto drive_train_command = get_node()->get_parameter("drive_train_command").as_string();
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = {"motor/" + drive_train_command, "steering/angle"};
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration TeleopController::state_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn TeleopController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn TeleopController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type TeleopController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    command_interfaces_[0].set_value(effort);
    command_interfaces_[1].set_value(steering_angle);
    return controller_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_controllers::TeleopController, controller_interface::ControllerInterface)