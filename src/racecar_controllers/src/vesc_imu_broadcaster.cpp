#include "racecar_controllers/vesc_imu_broadcaster.hpp"
#include "rclcpp/logging.hpp"

namespace racecar_controllers
{

  VescImuBroadcaster::VescImuBroadcaster() : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn VescImuBroadcaster::on_init()
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn VescImuBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration VescImuBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration VescImuBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = {
      "accelerometer/x",
      "accelerometer/y",
      "accelerometer/z",
      "gyroscope/x",
      "gyroscope/y",
      "gyroscope/z",
      "gyroscope/q.x",
      "gyroscope/q.y",
      "gyroscope/q.z",
      "gyroscope/q.w",
    };
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn VescImuBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (const auto& state_interface: state_interfaces_) {
      states_[state_interface.get_name()] = state_interface.get_value();
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn VescImuBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type VescImuBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    auto old_state = states_;
    for (const auto& state_interface: state_interfaces_) {
      states_[state_interface.get_name()] = state_interface.get_value();
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = time;
    msg.header.frame_id = "vesc";

    msg.orientation.x = states_["gyroscope/q.x"];
    msg.orientation.y = states_["gyroscope/q.y"];
    msg.orientation.z = states_["gyroscope/q.z"];
    msg.orientation.w = states_["gyroscope/q.w"];
    msg.orientation_covariance[0] = -1.0;

    msg.linear_acceleration.x = states_["accelerometer/x"];
    msg.linear_acceleration.y = states_["accelerometer/y"];
    msg.linear_acceleration.z = states_["accelerometer/z"];
    msg.linear_acceleration_covariance[0] = -1.0;

    msg.angular_velocity.x = (states_["gyroscope/x"] - old_state["gyroscope/x"]) / period.seconds();
    msg.angular_velocity.y = (states_["gyroscope/y"] - old_state["gyroscope/y"]) / period.seconds();
    msg.angular_velocity.z = (states_["gyroscope/z"] - old_state["gyroscope/z"]) / period.seconds();    
    msg.angular_velocity_covariance[0] = -1.0;
    
    publisher_->publish(msg);
    return controller_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_controllers::VescImuBroadcaster, controller_interface::ControllerInterface)