// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roam_bot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "roam_bot/arduino_comms.hpp"

std::vector<std::string> parse_string_vector(const std::string & input)
{
  std::vector<std::string> output;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(0, item.find_first_not_of(" \t\n\r")); // left trim
    item.erase(item.find_last_not_of(" \t\n\r") + 1); // right trim
    if (!item.empty()) {
      output.push_back(item);
    }
  }
  return output;
}


namespace roam_bot
{
hardware_interface::CallbackReturn RoamBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_names = parse_string_vector(
    info_.hardware_parameters.at("left_wheel_names"));
  cfg_.right_wheel_names = parse_string_vector(
    info_.hardware_parameters.at("right_wheel_names"));
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  // if (info_.hardware_parameters.count("pid_p") > 0)
  // {
  //   cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
  //   cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
  //   cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
  //   cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  // }

  wheel_lf_.setup(cfg_.left_wheel_names[0], cfg_.enc_counts_per_rev);
  wheel_lr_.setup(cfg_.left_wheel_names[1], cfg_.enc_counts_per_rev);

  wheel_rf_.setup(cfg_.right_wheel_names[0], cfg_.enc_counts_per_rev);
  wheel_rr_.setup(cfg_.right_wheel_names[1], cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  // for (int i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoamBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoamBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double delta_seconds = period.seconds();

  comms_.read_encoder_values(wheel_lf_.enc, wheel_rf_.enc);

  double lf_pos_prev = wheel_lf_.pos;
  double rf_pos_prev = wheel_rf_.pos;
  
  wheel_lf_.pos = wheel_lf_.calc_enc_angle();
  wheel_lf_.vel = (wheel_lf_.pos - lf_pos_prev) / delta_seconds;
  
  wheel_rf_.pos = wheel_rf_.calc_enc_angle();
  wheel_rf_.vel = (wheel_rf_.pos - rf_pos_prev) / delta_seconds;
  


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type roam_bot ::RoamBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int motor_lf_counts_per_loop = static_cast<int>(std::round(wheel_lf_.cmd / wheel_lf_.rads_per_count / cfg_.loop_rate ) ); 
  int motor_rf_counts_per_loop = static_cast<int>(std::round(wheel_rf_.cmd / wheel_rf_.rads_per_count / cfg_.loop_rate ) ); 
  int motor_lr_counts_per_loop = static_cast<int>(std::round(wheel_lr_.cmd / wheel_lr_.rads_per_count / cfg_.loop_rate ) ); 
  int motor_rr_counts_per_loop = static_cast<int>(std::round(wheel_rr_.cmd / wheel_rr_.rads_per_count / cfg_.loop_rate ) ); 

  comms_.set_motor_values(motor_lf_counts_per_loop, motor_rf_counts_per_loop, motor_lr_counts_per_loop, motor_rr_counts_per_loop);

  return hardware_interface::return_type::OK;
}

}  // namespace roam_bot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roam_bot::RoamBotSystemHardware, hardware_interface::SystemInterface)
