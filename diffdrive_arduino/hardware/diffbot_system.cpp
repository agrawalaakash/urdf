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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino {
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left1_wheel_name = info_.hardware_parameters["left1_wheel_name"];
  cfg_.right1_wheel_name = info_.hardware_parameters["right1_wheel_name"];
  cfg_.left2_wheel_name = info_.hardware_parameters["left2_wheel_name"];
  cfg_.right2_wheel_name = info_.hardware_parameters["right2_wheel_name"];
  cfg_.left3_wheel_name = info_.hardware_parameters["left3_wheel_name"];
  cfg_.right3_wheel_name = info_.hardware_parameters["right3_wheel_name"];
  cfg_.left4_wheel_name = info_.hardware_parameters["left4_wheel_name"];
  cfg_.right4_wheel_name = info_.hardware_parameters["right4_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev =
      std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0) {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
                "PID values not supplied, using defaults.");
  }

  wheel_l1_.setup(cfg_.left1_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r1_.setup(cfg_.right1_wheel_name, cfg_.enc_counts_per_rev);
  wheel_l2_.setup(cfg_.left2_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r2_.setup(cfg_.right2_wheel_name, cfg_.enc_counts_per_rev);
  wheel_l3_.setup(cfg_.left3_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r3_.setup(cfg_.right3_wheel_name, cfg_.enc_counts_per_rev);
  wheel_l4_.setup(cfg_.left4_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r4_.setup(cfg_.right4_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduinoHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("DiffDriveArduinoHardware"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("DiffDriveArduinoHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DiffDriveArduinoHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l1_.name, hardware_interface::HW_IF_POSITION, &wheel_l1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l1_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r1_.name, hardware_interface::HW_IF_POSITION, &wheel_r1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r1_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l2_.name, hardware_interface::HW_IF_POSITION, &wheel_l2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r2_.name, hardware_interface::HW_IF_POSITION, &wheel_r2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l3_.name, hardware_interface::HW_IF_POSITION, &wheel_l3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l3_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r3_.name, hardware_interface::HW_IF_POSITION, &wheel_r3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r3_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l4_.name, hardware_interface::HW_IF_POSITION, &wheel_l4_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_l4_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l4_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r4_.name, hardware_interface::HW_IF_POSITION, &wheel_r4_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_r4_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r4_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffDriveArduinoHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l1_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r1_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l2_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r2_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l3_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r3_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_l4_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l4_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_r4_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r4_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Configuring ...please wait...");
  if (comms_.connected()) {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Cleaning up ...please wait...");
  if (comms_.connected()) {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Activating ...please wait...");
  if (!comms_.connected()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0) {
    comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
DiffDriveArduinoHardware::read(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration &period) {
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_l1_.enc, wheel_r1_.enc, wheel_l2_.enc, wheel_r2_.enc, wheel_l3_.enc, wheel_r3_.enc, wheel_l4_.enc, wheel_r4_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l1_.pos = wheel_l1_.calc_enc_angle();
  wheel_l1_.vel = (wheel_l1_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r1_.pos = wheel_r1_.calc_enc_angle();
  wheel_r1_.vel = (wheel_r1_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_l_.pos;
  wheel_l2_.pos = wheel_l2_.calc_enc_angle();
  wheel_l2_.vel = (wheel_l2_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r2_.pos = wheel_r2_.calc_enc_angle();
  wheel_r2_.vel = (wheel_r2_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_l_.pos;
  wheel_l3_.pos = wheel_l3_.calc_enc_angle();
  wheel_l3_.vel = (wheel_l3_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r3_.pos = wheel_r3_.calc_enc_angle();
  wheel_r3_.vel = (wheel_r3_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_l_.pos;
  wheel_l4_.pos = wheel_l4_.calc_enc_angle();
  wheel_l4_.vel = (wheel_l4_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r4_.pos = wheel_r4_.calc_enc_angle();
  wheel_r4_.vel = (wheel_r4_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
diffdrive_arduino ::DiffDriveArduinoHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  int motor_l1_counts_per_loop =
      wheel_l1_.cmd / wheel_l1_.rads_per_count / cfg_.loop_rate;
  int motor_r1_counts_per_loop =
      wheel_r1_.cmd / wheel_r1_.rads_per_count / cfg_.loop_rate;
  int motor_l2_counts_per_loop =
      wheel_l2_.cmd / wheel_l2_.rads_per_count / cfg_.loop_rate;
  int motor_r2_counts_per_loop =
      wheel_r2_.cmd / wheel_r2_.rads_per_count / cfg_.loop_rate;
  int motor_l3_counts_per_loop =
      wheel_l3_.cmd / wheel_l3_.rads_per_count / cfg_.loop_rate;
  int motor_r3_counts_per_loop =
      wheel_r3_.cmd / wheel_r3_.rads_per_count / cfg_.loop_rate;
  int motor_l4_counts_per_loop =
      wheel_l4_.cmd / wheel_l4_.rads_per_count / cfg_.loop_rate;
  int motor_r4_counts_per_loop =
      wheel_r4_.cmd / wheel_r4_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_l1_counts_per_loop, motor_r1_counts_per_loop,
                          motor_l2_counts_per_loop, motor_r2_counts_per_loop,
                          motor_l3_counts_per_loop, motor_r3_counts_per_loop,
                          motor_l4_counts_per_loop, motor_r4_counts_per_loop);
  return hardware_interface::return_type::OK;
}

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::DiffDriveArduinoHardware,
                       hardware_interface::SystemInterface)
