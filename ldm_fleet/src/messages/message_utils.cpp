/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <dds/dds.h>

#include "../dds_utils/common.hpp"

#include "message_utils.hpp"

namespace ldm_fleet {
namespace messages {

void convert(const LiftRequest& _input, LdmFleetData_LiftRequest& _output)
{
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  _output.lift_name = common::dds_string_alloc_and_copy(_input.lift_name);
  _output.request_type = _input.request_type;
  _output.destination_floor = common::dds_string_alloc_and_copy(_input.destination_floor);
  _output.door_state = _input.door_state;

}

void convert(const LdmFleetData_LiftRequest& _input, LiftRequest& _output)
{
  _output.request_id =  std::string(_input.request_id);
  _output.lift_name = std::string(_input.lift_name);
  _output.request_type = _input.request_type;
  _output.destination_floor = std::string(_input.destination_floor);
  _output.door_state = _input.door_state;

}

void convert(const LiftState& _input, LdmFleetData_LiftState& _output)
{
  _output.lift_name = common::dds_string_alloc_and_copy(_input.lift_name);
  _output.current_floor = common::dds_string_alloc_and_copy(_input.current_floor);
  _output.door_state = _input.door_state;
  _output.motion_state = _input.motion_state;
  _output.current_mode = _input.current_mode;
  _output.register_state = _input.register_state;
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
}

void convert(const LdmFleetData_LiftState& _input, LiftState& _output)
{
  _output.lift_name = std::string(_input.lift_name);
  _output.current_floor = std::string(_input.current_floor);
  _output.door_state = _input.door_state;
  _output.motion_state = _input.motion_state;
  _output.current_mode = _input.current_mode;
  _output.register_state = _input.register_state;
  _output.request_id = std::string(_input.request_id);
}

void convert(const RegisterRequest& _input, LdmFleetData_RegisterRequest& _output)
{
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  _output.device_name = common::dds_string_alloc_and_copy(_input.device_name);
  _output.device_type = _input.device_type;
  _output.register_mode = _input.register_mode;

}

void convert(const LdmFleetData_RegisterRequest& _input, RegisterRequest& _output)
{
  _output.request_id =  std::string(_input.request_id);
  _output.device_name = std::string(_input.device_name);
  _output.device_type = _input.device_type;
  _output.register_mode = _input.register_mode;

}

} // namespace messages
} // namespace ldm_fleet
