#include "racecar_controllers/teleop_controller.hpp"
#include "rclcpp/logging.hpp"

namespace racecar_controllers
{

  TeleopController::TeleopController() : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn TeleopController::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "on init");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn TeleopController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_node()->get_logger(), "on configure");
    auto callback = [this](const sensor_msgs::msg::Joy &msg)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Effort: %f", msg.axes[1]);
      auto drive_train_command = this->get_node()->get_parameter("drive_train_command").as_string();
      if (drive_train_command == "current")
      {
        this->effort = msg.axes[1] * 60.0;
      }
      else if (drive_train_command == "duty_cycle") {
        this->effort = msg.axes[1];
      } else
      {
        this->effort = msg.axes[1] * 30000.0;
      }
      this->steering_angle = msg.axes[3];
    };
    command_sub = this->get_node()->create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration TeleopController::command_interface_configuration() const
  {
    RCLCPP_INFO(get_node()->get_logger(), "command interface configuration");
    auto drive_train_command = get_node()->get_parameter("drive_train_command").as_string();
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = {"motor/" + drive_train_command, "steering/angle"};
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration TeleopController::state_interface_configuration() const
  {
    RCLCPP_INFO(get_node()->get_logger(), "state interface configuration");
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn TeleopController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "on activate");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn TeleopController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "on deactivate");
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