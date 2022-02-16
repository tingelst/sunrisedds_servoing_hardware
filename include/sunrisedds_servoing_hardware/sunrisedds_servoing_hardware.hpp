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

/* Original Author: Lars Tingelstad
 * Co-Author: Mathias Hauan Arbo
 */

#ifndef SUNRISEDDS_SERVOING_HARDWARE__SUNRISEDDS_SERVOING_HARDWARE_HPP_
#define SUNRISEDDS_SERVOING_HARDWARE__SUNRISEDDS_SERVOING_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <dds/dds.h>

#include "rclcpp/macros.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "sunrisedds_servoing_hardware/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace sunrisedds_servoing_hardware
{

static constexpr int MAX_SAMPLES = 1;

class SunriseDdsServoingHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SunriseDdsServoingHardware)

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  SUNRISEDDS_SERVOING_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

  ~SunriseDdsServoingHardware();

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  dds_entity_t participant_;
  dds_entity_t command_topic_;
  dds_entity_t state_topic_;
  dds_entity_t writer_;
  dds_entity_t reader_;
  dds_entity_t read_condition_;
  dds_entity_t waitset_;

  void* samples_[MAX_SAMPLES];
  dds_sample_info_t infos_[MAX_SAMPLES];

};

}  // namespace sunrisedds_servoing_hardware
#endif  // SUNRISEDDS_SERVOING_HARDWARE__SUNRISEDDS_SERVOING_HARDWARE_HPP_
