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

#include <chrono>

#include <Eigen/Geometry>

#include <ldm_fleet/Server.hpp>
#include <ldm_fleet/ServerConfig.hpp>

#include <ldm_fleet/messages/LiftRequest.hpp>
#include <ldm_fleet/messages/LiftState.hpp>


#include "utilities.hpp"
#include "ServerNode.hpp"

namespace ldm_fleet
{
namespace ros2
{

ServerNode::SharedPtr ServerNode::make(
    const ServerNodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  // Starting the ldm fleet server node
  SharedPtr server_node(new ServerNode(_config, _node_options));

  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();
  while (
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count() < 10)
  {
    rclcpp::spin_some(server_node);

    server_node->setup_config();
    if (server_node->is_ready())
      break;
    RCLCPP_INFO(
        server_node->get_logger(), "waiting for configuration parameters.");

    end_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!server_node->is_ready())
  {
    RCLCPP_ERROR(
        server_node->get_logger(), "unable to initialize parameters.");
    return nullptr;
  }
  server_node->print_config();

  // Starting the ldm fleet server
  ServerConfig server_config =
      server_node->server_node_config.get_server_config();
  Server::SharedPtr server = Server::make(server_config);
  if (!server)
    return nullptr;

  server_node->start(Fields{
    std::move(server)
  });

  return server_node;
}

ServerNode::~ServerNode()
{}

ServerNode::ServerNode(
    const ServerNodeConfig& _config,
    const rclcpp::NodeOptions& _node_options) :
  Node("ldm_fleet_server_node", _node_options),
  server_node_config(_config)
{}

void ServerNode::print_config()
{
  server_node_config.print_config();
}

void ServerNode::setup_config()
{
  get_parameter("fleet_state_topic", server_node_config.fleet_state_topic);
  get_parameter("lift_request_topic", server_node_config.lift_request_topic);
  get_parameter("dds_domain", server_node_config.dds_domain);
  get_parameter("dds_lift_state_topic", server_node_config.dds_lift_state_topic);
  get_parameter("dds_lift_request_topic", server_node_config.dds_lift_request_topic);
  get_parameter("update_state_frequency", server_node_config.update_state_frequency);
  get_parameter("publish_state_frequency", server_node_config.publish_state_frequency);
}

bool ServerNode::is_ready()
{
  // if (server_node_config.fleet_name == "fleet_name")
  //   return false;
  return true;
}

void ServerNode::start(Fields _fields)
{
  fields = std::move(_fields);

  {
    WriteLock ldm_states_lock(ldm_states_mutex);
    lift_states.clear();
  }

  using namespace std::chrono_literals;

  // --------------------------------------------------------------------------
  // First callback group that handles getting updates from all the clients
  // available

  update_state_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  update_state_timer = create_wall_timer(
      100ms, std::bind(&ServerNode::update_state_callback, this),
      update_state_callback_group);

  // --------------------------------------------------------------------------
  // Second callback group that handles publishing fleet states to RMF, and
  // handling requests from RMF to be sent down to the clients

  fleet_state_pub_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  fleet_state_pub =
      create_publisher<ldm_fleet_msgs::msg::FleetLiftState>(
          server_node_config.fleet_state_topic, 10);


  fleet_state_pub_timer = create_wall_timer(
      std::chrono::seconds(1) / server_node_config.publish_state_frequency,
      std::bind(&ServerNode::publish_fleet_state, this),
      fleet_state_pub_callback_group);

  // --------------------------------------------------------------------------
  // Lift request handling

  auto lift_request_sub_opt = rclcpp::SubscriptionOptions();

  lift_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  lift_request_sub = create_subscription<ldm_fleet_msgs::msg::LiftRequest>(
      server_node_config.lift_request_topic, rclcpp::QoS(10),
      [&](ldm_fleet_msgs::msg::LiftRequest::UniquePtr msg)
      {
        handle_lift_request(std::move(msg));
      },
      lift_request_sub_opt);

  // --------------------------------------------------------------------------
}

bool ServerNode::is_request_valid( const std::string& _lift_name)
{
  // if (_fleet_name != server_node_config.fleet_name)
  //   return false;

  ReadLock ldm_states_lock(ldm_states_mutex);
  auto it = lift_states.find(_lift_name);
  if (it == lift_states.end())
    return false;
  return true;
}

void ServerNode::handle_lift_request(
    ldm_fleet_msgs::msg::LiftRequest::UniquePtr _msg)
{
  messages::LiftRequest ldmf_msg;
  to_ldmf_message(*(_msg.get()), ldmf_msg);
  fields.server->send_lift_request(ldmf_msg);
}

void ServerNode::update_state_callback()
{
  // Update State
  std::vector<messages::LiftState> new_lift_states;
  fields.server->read_lift_states(new_lift_states);

  for (const messages::LiftState& mf_ms : new_lift_states)
  {
    ldm_fleet_msgs::msg::LiftState ros_ms;
    to_ros_message(mf_ms, ros_ms);

    WriteLock ldm_states_lock(ldm_states_mutex);
    auto it = lift_states.find(ros_ms.lift_name);
    if (it == lift_states.end())
      RCLCPP_INFO(
          get_logger(),
          "registered a new lift: [%s]",
          ros_ms.lift_name.c_str());

    lift_states[ros_ms.lift_name] = ros_ms;
  }
}

void ServerNode::publish_fleet_state()
{
  ldm_fleet_msgs::msg::FleetLiftState fleet_state;
  // fleet_state.name = server_node_config.fleet_name;
  fleet_state.lifts.clear();

  ReadLock ldm_states_lock(ldm_states_mutex);
  for (const auto it : lift_states)
  {
    const auto fleet_frame_rs = it.second;
    ldm_fleet_msgs::msg::LiftState rmf_frame_ldms;

    rmf_frame_ldms.lift_name = fleet_frame_rs.lift_name;
    rmf_frame_ldms.current_floor = fleet_frame_rs.current_floor;
    rmf_frame_ldms.door_state = fleet_frame_rs.door_state;
    rmf_frame_ldms.motion_state = fleet_frame_rs.motion_state;
    rmf_frame_ldms.current_mode = fleet_frame_rs.current_mode;
    rmf_frame_ldms.request_id = fleet_frame_rs.request_id;
    fleet_state.lifts.push_back(rmf_frame_ldms);
  }
  fleet_state_pub->publish(fleet_state);
}

} // namespace ros2
} // namespace ldm_fleet
