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

#include "utilities.hpp"

namespace ldm_fleet
{
namespace ros2
{

void to_ldmf_message(
    const ldm_fleet_msgs::msg::LiftRequest& _in_msg, 
    messages::LiftRequest& _out_msg)
{
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.lift_name = _in_msg.lift_name;
  _out_msg.request_type = _in_msg.request_type;
  _out_msg.destination_floor = _in_msg.destination_floor;
  _out_msg.door_state = _in_msg.door_state;
}

void to_ros_message(
    const messages::LiftState& _in_msg,
    ldm_fleet_msgs::msg::LiftState& _out_msg)
{
  _out_msg.lift_name = _in_msg.lift_name;
  _out_msg.current_floor = _in_msg.current_floor;
  _out_msg.door_state = _in_msg.door_state;
  _out_msg.motion_state = _in_msg.motion_state;
  _out_msg.current_mode = _in_msg.current_mode;
  _out_msg.request_id = _in_msg.request_id;
}

} // namespace ros2
} // namespace ldm_fleet
