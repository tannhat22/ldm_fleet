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

#ifndef LDM_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
#define LDM_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP

#include <ldm_fleet_msgs/msg/lift_request.hpp>
#include <ldm_fleet_msgs/msg/lift_state.hpp>
#include <ldm_fleet_msgs/msg/register_request.hpp>


#include <ldm_fleet/messages/LiftRequest.hpp>
#include <ldm_fleet/messages/LiftState.hpp>
#include <ldm_fleet/messages/RegisterRequest.hpp>

namespace ldm_fleet
{
namespace ros2
{

void to_ldmf_message(
    const ldm_fleet_msgs::msg::LiftRequest& in_msg, 
    messages::LiftRequest& out_msg);

void to_ldmf_message(
    const ldm_fleet_msgs::msg::RegisterRequest& in_msg,
    messages::RegisterRequest& out_msg);

// ----------------------------------------------------------------------------

void to_ros_message(
    const messages::LiftState& in_msg,
    ldm_fleet_msgs::msg::LiftState& out_msg);

} // namespace ros2
} // namespace ldm_fleet


#endif // LDM_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
