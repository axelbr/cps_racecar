#include "racecar_hardware/vesc_hardware_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cassert>
#include <sstream>
#include <string>
#include <exception>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace racecar_hardware
{
  hardware_interface::CallbackReturn VescHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Set hardware config
    config_.hw_port = info_.hardware_parameters["vesc_port"];
    config_.erpm_gain = std::stod(info_.hardware_parameters["erpm_gain"]);
    config_.erpm_offset = std::stod(info_.hardware_parameters["erpm_offset"]);
    config_.servo_gain = std::stod(info_.hardware_parameters["servo_gain"]);
    config_.servo_offset = std::stod(info_.hardware_parameters["servo_offset"]);
    config_.activate_current = std::stoi(info_.hardware_parameters["use_current"]);
    config_.activate_duty_cycle = std::stoi(info_.hardware_parameters["use_duty_cycle"]);
    config_.activate_erpm = std::stoi(info_.hardware_parameters["use_erpm"]);

    // Check joint and component info
    if (info.joints.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("VescHardwareInterface"), "Expected 2 joints: [drive_train, steering]");
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &drive_train_joint = info.joints[0];
    const auto &steering_joint = info.joints[1];
    if (not check_drive_train_info(drive_train_joint) or not check_steering_info(steering_joint))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Set up the maps for each joint and sensor.
    set_interface_fields(info);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void VescHardwareInterface::set_interface_fields(const hardware_interface::HardwareInfo &info)
  {
    // For each joint...
    for (const auto &joint : info.joints)
    {

      // ...set command limits and initial values...
      for (const auto &command : joint.command_interfaces)
      {
        ControlInterface control;
        double min = std::stod(command.min);
        double max = std::stod(command.max);
        control.value = std::clamp(0.0, min, max);
        control.limits = std::make_pair(min, max);
        controls_[joint.name][command.name] = control;
      }

      // ...and state interface values.
      for (const auto &state : joint.state_interfaces)
      {
        states_[joint.name][state.name] = 0.0; // std::stod(state.initial_value);
      }
    }

    // For each sensor, define an entry in the state interface map.
    for (const auto &sensor : info.sensors)
    {
      for (const auto &state : sensor.state_interfaces)
      {
        states_[sensor.name][state.name] = 0.0; // std::stod(state.initial_value);
      }
    }
  }

  bool VescHardwareInterface::check_drive_train_info(const hardware_interface::ComponentInfo &info)
  {
    return true;
  }

  bool VescHardwareInterface::check_steering_info(const hardware_interface::ComponentInfo &info)
  {
    return true;
  }

  void VescHardwareInterface::packet_callback(const std::shared_ptr<vesc_driver::VescPacket const> &packet)
  {
    if (packet->name() == "Values")
    {
      packet_data_.motor_data = std::dynamic_pointer_cast<vesc_driver::VescPacketValues const>(packet);
    }
    else if (packet->name() == "FWVersion")
    {
      packet_data_.firmware_data = std::dynamic_pointer_cast<vesc_driver::VescPacketFWVersion const>(packet);
    }
    else if (packet->name() == "ImuData")
    {
      packet_data_.imu_data = std::dynamic_pointer_cast<vesc_driver::VescPacketImu const>(packet);
    }
  }

  void VescHardwareInterface::error_callback(const std::string &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("VescHardwareInterface"), "Received error from VESC: %s", error.c_str());
  }

  std::vector<hardware_interface::StateInterface> VescHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const auto &joint : info_.joints)
    {
      for (const auto &state : joint.state_interfaces)
      {
        hardware_interface::StateInterface state_interface(joint.name, state.name, &states_[joint.name][state.name]);
        state_interfaces.emplace_back(state_interface);
      }
    }

    for (const auto &sensor : info_.sensors)
    {
      for (const auto &state : sensor.state_interfaces)
      {
        hardware_interface::StateInterface state_interface(sensor.name, state.name, &states_[sensor.name][state.name]);
        state_interfaces.emplace_back(state_interface);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Exporting state interfaces.");
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> VescHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (const auto &joint : info_.joints)
    {
      for (const auto &command : joint.command_interfaces)
      {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, command.name, &controls_[joint.name][command.name].value));
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Exporting command interfaces.");
    return command_interfaces;
  }

  hardware_interface::CallbackReturn VescHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    try
    {
      vesc_.connect(config_.hw_port);
      vesc_.setPacketHandler([this](const auto &packet)
                             { this->packet_callback(packet); });
      vesc_.setErrorHandler([this](const auto &error)
                            { this->error_callback(error); });
      vesc_.requestFWVersion();
      vesc_.requestState();
      vesc_.requestImuData();
    }
    catch (std::exception &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("VescHardwareInterface"), "Failed to connect to the VESC: %s.", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Successfully connected to VESC.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn VescHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    vesc_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Disconnected from VESC.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type VescHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    states_["drive_train"]["erpm"] = packet_data_.motor_data->rpm();
    states_["drive_train"]["velocity"] = (packet_data_.motor_data->rpm() - config_.erpm_offset) / config_.erpm_gain;
    states_["drive_train"]["duty_cycle"] = packet_data_.motor_data->duty_cycle_now();
    states_["drive_train"]["current_motor"] = packet_data_.motor_data->avg_motor_current();
    states_["drive_train"]["current_input"] = packet_data_.motor_data->avg_input_current();
    states_["drive_train"]["voltage"] = packet_data_.motor_data->v_in();
    states_["drive_train"]["charge_drawn"] = packet_data_.motor_data->amp_hours();
    states_["drive_train"]["charge_regenerated"] = packet_data_.motor_data->amp_hours_charged();
    states_["drive_train"]["displacement"] = packet_data_.motor_data->tachometer();
    states_["drive_train"]["distance_travelled"] = packet_data_.motor_data->tachometer_abs();

    states_["steering"]["angle"] = controls_["steering"]["angle"].value;

    states_["gyroscope"]["yaw"] = packet_data_.imu_data->yaw();
    states_["gyroscope"]["pitch"] = packet_data_.imu_data->pitch();
    states_["gyroscope"]["roll"] = packet_data_.imu_data->roll();
    states_["gyroscope"]["x"] = packet_data_.imu_data->gyr_x();
    states_["gyroscope"]["y"] = packet_data_.imu_data->gyr_y();
    states_["gyroscope"]["z"] = packet_data_.imu_data->gyr_z();
    states_["gyroscope"]["q.x"] = packet_data_.imu_data->q_x();
    states_["gyroscope"]["q.y"] = packet_data_.imu_data->q_y();
    states_["gyroscope"]["q.z"] = packet_data_.imu_data->q_z();
    states_["gyroscope"]["q.w"] = packet_data_.imu_data->q_w();

    states_["accelerometer"]["x"] = packet_data_.imu_data->acc_x();
    states_["accelerometer"]["y"] = packet_data_.imu_data->acc_y();
    states_["accelerometer"]["z"] = packet_data_.imu_data->acc_z();

    states_["magnetometer"]["x"] = packet_data_.imu_data->mag_x();
    states_["magnetometer"]["y"] = packet_data_.imu_data->mag_y();
    states_["magnetometer"]["z"] = packet_data_.imu_data->mag_z();

    vesc_.requestState();
    vesc_.requestImuData();

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type racecar_hardware::VescHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    auto enforce_limits = [](const ControlInterface &control)
    {
      auto [min, max] = control.limits;
      return std::clamp(control.value, min, max);
    };

    auto print = [](std::map<std::string, std::map<std::string, ControlInterface>> map)
    {
      for (auto itr1 = map.begin(); itr1 != map.end(); itr1++)
      {
        std::cout << itr1->first << ' '; // Add space to separate entries on the same line
        for (auto itr2 = itr1->second.begin(); itr2 != itr1->second.end(); itr2++)
        {
          std::cout << itr2->first << " (" << itr2->second.value << ", " << itr2->second.limits.first << ", " << itr2->second.limits.second << ") " << std::endl;
        }
        std::cout << std::endl;
      }
    };

    // target_velocity_ * erpm_gain_ + erpm_offset_;
    // print(controls_);
    if (config_.activate_erpm)
    {
      auto erpm = enforce_limits(controls_["drive_train"]["erpm"]);
      vesc_.setSpeed(erpm);
    }
    if (config_.activate_current)
    {
      auto current = enforce_limits(controls_["drive_train"]["current"]);
      vesc_.setCurrent(current);
    }

    if (config_.activate_duty_cycle)
    {
      auto duty_cycle = enforce_limits(controls_["drive_train"]["duty_cycle"]);
      vesc_.setDutyCycle(duty_cycle);
    }
    
    auto angle = enforce_limits(controls_["steering"]["angle"]);
    vesc_.setServo(config_.servo_gain * angle + config_.servo_offset);
    
    return hardware_interface::return_type::OK;
  }

} // namespace racecar_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_hardware::VescHardwareInterface, hardware_interface::SystemInterface)