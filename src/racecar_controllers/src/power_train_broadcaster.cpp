#include "racecar_controllers/power_train_broadcaster.hpp"
#include "rclcpp/logging.hpp"

namespace racecar_controllers
{

  PowerTrainBroadcaster::PowerTrainBroadcaster() : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn PowerTrainBroadcaster::on_init()
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn PowerTrainBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    power_train_publisher_ = get_node()->create_publisher<racecar_msgs::msg::PowerTrainState>("~/powertrain", rclcpp::SystemDefaultsQoS());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration PowerTrainBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration PowerTrainBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = {
      "motor/rpm",
      "motor/duty_cycle",
      "motor/current_control",
      "motor/current_motor",
      "motor/current_input",
      "motor/voltage",
      "motor/charge_drawn",
      "motor/charge_regenerated",
      "power_train_feedback/wheels_rpm",
      "power_train_feedback/speed",
      "steering/angle"
    };
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn PowerTrainBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
   
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn PowerTrainBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "on deactivate");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type PowerTrainBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
  {
    for (const auto& state_interface: state_interfaces_) {
      states_[state_interface.get_name()] = state_interface.get_value();
    }
    auto msg = racecar_msgs::msg::PowerTrainState();
    msg.header.stamp = time;
    for (int i = 0; i < msg.wheel_rpms.size(); i++) {
      msg.wheel_rpms[i] = states_["power_train_feedback/wheels_rpm"];
    }
    msg.speed = states_["power_train_feedback/speed"];
    msg.motor_current = states_["motor/current_motor"];
    msg.motor_rpm = states_["motor/rpm"];
    msg.input_current = states_["motor/current_input"];
    msg.command_current = states_["motor/current_control"];
    msg.duty_cycle = states_["motor/duty_cycle"];
    msg.voltage = states_["motor/voltage"];
    msg.charge_drawn = states_["motor/charge_drawn"];
    msg.charge_regenerated = states_["motor/charge_regenerated"];
    msg.steering_angle = states_["steering/angle"];
    power_train_publisher_->publish(msg);
    return controller_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_controllers::PowerTrainBroadcaster, controller_interface::ControllerInterface)