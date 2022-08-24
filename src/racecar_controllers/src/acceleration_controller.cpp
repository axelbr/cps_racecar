#include "racecar_controllers/acceleration_controller.hpp"
#include "rclcpp/logging.hpp"

namespace racecar_controllers
{

  AccelerationController::AccelerationController() : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn AccelerationController::on_init()
  {
    try
    {
      auto_declare<std::string>("command_interface_name", params_.command_interface_name);
      auto_declare<std::string>("acceleration_sensor_name", params_.acceleration_sensor_name);
      auto_declare<std::string>("velocity_sensor_name", params_.velocity_sensor_name);

      auto_declare<double>("acceleration_control.P", params_.acceleration_controller_params.P);
      auto_declare<double>("acceleration_control.I", params_.acceleration_controller_params.I);
      auto_declare<double>("acceleration_control.D", params_.acceleration_controller_params.D);
      auto_declare<double>("acceleration_control.max", params_.acceleration_controller_params.max);
      auto_declare<double>("acceleration_control.min", params_.acceleration_controller_params.min);

      auto_declare<double>("velocity_control.P", params_.velocity_controller_params.P);
      auto_declare<double>("velocity_control.I", params_.velocity_controller_params.I);
      auto_declare<double>("velocity_control.D", params_.velocity_controller_params.D);
      auto_declare<double>("velocity_control.max", params_.velocity_controller_params.max);
      auto_declare<double>("velocity_control.min", params_.velocity_controller_params.min);
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AccelerationController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration AccelerationController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names.push_back(params_.command_interface_name);
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration AccelerationController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.push_back(params_.acceleration_sensor_name);
    state_interfaces_config.names.push_back(params_.velocity_sensor_name);
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn AccelerationController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
   
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AccelerationController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "on deactivate");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type AccelerationController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return controller_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_controllers::AccelerationController, controller_interface::ControllerInterface)