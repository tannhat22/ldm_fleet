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

#ifndef LDM_FLEET__ROS2__CLIENTNODE_HPP
#define LDM_FLEET__ROS2__CLIENTNODE_HPP

#include <deque>
#include <shared_mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ldm_fleet_msgs/srv/lift.hpp>
#include <ldm_fleet_msgs/msg/lift_state.hpp>
#include <ldm_fleet_msgs/msg/register_request.hpp>

#include <ldm_fleet/Client.hpp>
#include <ldm_fleet/ClientConfig.hpp>
#include <ldm_fleet/messages/LiftState.hpp>
#include <ldm_fleet/messages/LiftRequest.hpp>
#include <ldm_fleet/messages/RegisterRequest.hpp>

#include <ldm_fleet/Client.hpp>

#include "ldm_fleet/ros2/client_node_config.hpp"

namespace ldm_fleet
{
namespace ros2
{

class ClientNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<ClientNode>;
  using Mutex = std::mutex;
  using ReadLock = std::unique_lock<Mutex>;
  using WriteLock = std::unique_lock<Mutex>;

  explicit ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ClientNode() override;

  struct Fields
  {
    /// Ldm fleet client
    Client::SharedPtr client;

    // Lift server client
    rclcpp::Client<ldm_fleet_msgs::srv::Lift>::SharedPtr lift_trigger_client;
  };

  void print_config();

private:
  // --------------------------------------------------------------------------
  // Lift state handling

  rclcpp::Subscription<ldm_fleet_msgs::msg::LiftState>::SharedPtr  lift_state_sub;
  Mutex ldm_state_mutex;
  ldm_fleet_msgs::msg::LiftState current_lift_state;
  void lift_state_callback_fn(const ldm_fleet_msgs::msg::LiftState::SharedPtr msg);

  // --------------------------------------------------------------------------
  // Publisher:
  rclcpp::Publisher<ldm_fleet_msgs::msg::RegisterRequest>::SharedPtr register_req_pub;

  // TODO: conditions to trigger emergency, however this is most likely for
  // indicating emergency within the fleet and not in RMF
  // TODO: figure out a better way to handle multiple triggered modes
  std::atomic<bool> request_error;

  messages::LiftState get_lift_state();
  bool read_lift_request();
  
  bool read_register_request();

  // Request handling

  bool is_valid_request(
      const std::string& request_lift_name,
      const std::string& request_id);

  Mutex request_id_mutex;
  std::string current_request_id;

  // Mutex request_robot_mutex;
  // std::string current_request_robot_name;


  void read_requests();
  void handle_requests();
  void publish_lift_state();

  // --------------------------------------------------------------------------
  // publish and update functions and timers

  std::shared_ptr<rclcpp::TimerBase> update_timer;
  std::shared_ptr<rclcpp::TimerBase> publish_timer;
  void update_fn();
  void publish_fn();

  // --------------------------------------------------------------------------

  ClientNodeConfig client_node_config;
  Fields fields;

  void start(Fields fields);
};

} // namespace ros2
} // namespace ldm_fleet

#endif // LDM_FLEET__ROS2__CLIENTNODE_HPP