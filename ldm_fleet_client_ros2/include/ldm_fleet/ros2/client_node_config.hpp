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

#ifndef LDM_FLEET__ROS2__CLIENTNODECONFIG_HPP
#define LDM_FLEET__ROS2__CLIENTNODECONFIG_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ldm_fleet/ClientConfig.hpp>

namespace ldm_fleet
{
namespace ros2
{

struct ClientNodeConfig
{

  std::string lift_name = "lift_name";
  // std::string fleet_name = "fleet_name";

  std::string lift_state_topic = "/lift_state";
  std::string lift_trigger_server_name = "lift_server";

  int dds_domain = 52;
  std::string dds_state_topic = "lift_state";
  std::string dds_lift_request_topic = "lift_request";

  double wait_timeout = 10.0;
  double update_frequency = 10.0;
  double publish_frequency = 1.0;

  void print_config() const;

  ClientConfig get_client_config() const;
};

} // namespace ros2
} // namespace ldm_fleet

#endif // LDM_FLEET__ROS2__CLIENTNODECONFIG_HPP