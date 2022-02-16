// Copyright 2021 Norwegian University of Science and Technology.
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

/*
 * Author: Lars Tingelstad
 */

#include "sunrisedds_servoing_hardware/sunrisedds_servoing_hardware.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "JointPosition.h"

namespace sunrisedds_servoing_hardware
{
CallbackReturn SunriseDdsServoingHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    // KUKA RSI hardware has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "Joint '%s' has %zu state interface. 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"),
                   "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  participant_ = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_create_participant: %s",
                 dds_strretcode(-participant_));
    return CallbackReturn::ERROR;
  }

  command_topic_ =
      dds_create_topic(participant_, &sunrisedds_interfaces_msg_JointPosition_desc, "rt/command", nullptr, nullptr);
  if (command_topic_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_create_topic: %s",
                 dds_strretcode(-participant_));
    return CallbackReturn::ERROR;
  }

  state_topic_ =
      dds_create_topic(participant_, &sunrisedds_interfaces_msg_JointPosition_desc, "rt/state", nullptr, nullptr);
  if (state_topic_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_create_topic: %s",
                 dds_strretcode(-participant_));
    return CallbackReturn::ERROR;
  }

  writer_ = dds_create_writer(participant_, command_topic_, nullptr, nullptr);
  if (writer_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_create_writer: %s",
                 dds_strretcode(-participant_));
    return CallbackReturn::ERROR;
  }

  reader_ = dds_create_reader(participant_, command_topic_, nullptr, nullptr);
  if (reader_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_create_reader: %s",
                 dds_strretcode(-participant_));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SunriseDdsServoingHardware::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SunriseDdsServoingHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SunriseDdsServoingHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn SunriseDdsServoingHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  return CallbackReturn::SUCCESS;
}

CallbackReturn SunriseDdsServoingHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SunriseDdsServoingHardware::read()
{
  assert(hw_states_.size() == 7);

  void* samples[1];
  dds_sample_info_t infos[1];

  samples[0] = sunrisedds_interfaces_msg_JointPosition__alloc();

  dds_return_t ret = dds_take(reader_, samples, infos, 1, 1);
  if (ret < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_take: %s", dds_strretcode(-ret));
    return hardware_interface::return_type::ERROR;
  }

  if ((ret > 0) && (infos[0].valid_data))
  {
    sunrisedds_interfaces_msg_JointPosition* msg =
        reinterpret_cast<sunrisedds_interfaces_msg_JointPosition*>(samples[0]);

    hw_states_[0] = msg->position.a1;
    hw_states_[1] = msg->position.a2;
    hw_states_[2] = msg->position.a3;
    hw_states_[3] = msg->position.a4;
    hw_states_[4] = msg->position.a5;
    hw_states_[5] = msg->position.a6;
    hw_states_[6] = msg->position.a7;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SunriseDdsServoingHardware::write()
{
  assert(hw_commands_.size() == 7);

  sunrisedds_interfaces_msg_JointPosition msg;
  msg.position.a1 = hw_commands_[0];
  msg.position.a2 = hw_commands_[1];
  msg.position.a3 = hw_commands_[2];
  msg.position.a4 = hw_commands_[3];
  msg.position.a5 = hw_commands_[4];
  msg.position.a6 = hw_commands_[5];
  msg.position.a7 = hw_commands_[6];

  dds_return_t ret = dds_write(writer_, &msg);
  if (ret != DDS_RETCODE_OK)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_write: %s", dds_strretcode(-ret));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

SunriseDdsServoingHardware::~SunriseDdsServoingHardware()
{
  dds_return_t ret = dds_delete(participant_);
  if (ret != DDS_RETCODE_OK)
  {
    RCLCPP_FATAL(rclcpp::get_logger("SunriseDdsServoingHardware"), "dds_delete: %s", dds_strretcode(-ret));
  }
}

}  // namespace sunrisedds_servoing_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sunrisedds_servoing_hardware::SunriseDdsServoingHardware, hardware_interface::SystemInterface)
