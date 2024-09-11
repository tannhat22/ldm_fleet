/*
 * Copyright (C) 2019 Open Source chargerics Foundation
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

#include <iostream>
#include <exception>
#include <thread>

#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <ldm_fleet_msgs/srv/lift.hpp>

#include "ldm_fleet/ros2/client_node.hpp"
#include "ldm_fleet/ros2/client_node_config.hpp"

namespace ldm_fleet
{
namespace ros2
{
ClientNode::ClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ldm_fleet_client_ros2", options)
{
  /// Starting the ldm fleet client
  RCLCPP_INFO(get_logger(), "Greetings from %s", get_name());

  // parameter declarations
  // defaults declared in header
  declare_parameter("lift_name", client_node_config.lift_name);
  declare_parameter("lift_state_topic", client_node_config.lift_state_topic);
  declare_parameter("register_lift_topic", client_node_config.register_lift_topic);
  declare_parameter("lift_trigger_server_name", client_node_config.lift_trigger_server_name);
  declare_parameter("dds_domain", client_node_config.dds_domain);
  declare_parameter("dds_state_topic", client_node_config.dds_state_topic);
  declare_parameter("dds_lift_request_topic", client_node_config.dds_lift_request_topic);
  declare_parameter("dds_register_request_topic", client_node_config.dds_register_request_topic);
  declare_parameter("wait_timeout", client_node_config.wait_timeout);
  declare_parameter("update_frequency", client_node_config.update_frequency);
  declare_parameter("publish_frequency", client_node_config.publish_frequency);

  // getting new values for parameters or keep defaults
  get_parameter("lift_name", client_node_config.lift_name);
  get_parameter("lift_state_topic", client_node_config.lift_state_topic);
  get_parameter("register_lift_topic", client_node_config.register_lift_topic);
  get_parameter("lift_trigger_server_name", client_node_config.lift_trigger_server_name);
  get_parameter("dds_domain", client_node_config.dds_domain);
  get_parameter("dds_state_topic", client_node_config.dds_state_topic);
  get_parameter("dds_lift_request_topic", client_node_config.dds_lift_request_topic);
  get_parameter("dds_register_request_topic", client_node_config.dds_register_request_topic);
  get_parameter("wait_timeout", client_node_config.wait_timeout);
  get_parameter("update_frequency", client_node_config.update_frequency);
  get_parameter("publish_frequency", client_node_config.publish_frequency);
  print_config();

  ClientConfig client_config = client_node_config.get_client_config();
  Client::SharedPtr client = Client::make(client_config);
  if (!client) {
    throw std::runtime_error("Unable to create ldm_fleet Client from config.");
  }

  /// Setting up the lift service client, if required, wait for server
  rclcpp::Client<ldm_fleet_msgs::srv::Lift>::SharedPtr lift_trigger_client = nullptr;
  if (client_node_config.lift_trigger_server_name != "") {
    lift_trigger_client = create_client<ldm_fleet_msgs::srv::Lift>(
      client_node_config.lift_trigger_server_name);
    RCLCPP_INFO(
      get_logger(), "waiting for connection with lift trigger server: %s",
      client_node_config.lift_trigger_server_name.c_str());
    while (!lift_trigger_client->wait_for_service(
        std::chrono::duration<double>(client_node_config.wait_timeout)))
    {
      RCLCPP_ERROR(
        get_logger(), "timed out waiting for lift trigger server: %s",
        client_node_config.lift_trigger_server_name.c_str());
      if (!rclcpp::ok()) {
        throw std::runtime_error("exited rclcpp while constructing client_node");
      }
    }
  }

  start(
    Fields{
        std::move(client),
        std::move(lift_trigger_client)
      });
}

ClientNode::~ClientNode()
{
}

void ClientNode::start(Fields _fields)
{
  fields = std::move(_fields);

  // Subcribers:
  lift_state_sub = create_subscription<ldm_fleet_msgs::msg::LiftState>(
    client_node_config.lift_state_topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ClientNode::lift_state_callback_fn, this, std::placeholders::_1));

  // Publishers:
  register_req_pub = create_publisher<ldm_fleet_msgs::msg::RegisterRequest>(
    client_node_config.register_lift_topic, 10);

  request_error = false;

  RCLCPP_INFO(get_logger(), "starting update timer.");
  std::chrono::duration<double> update_period =
    std::chrono::duration<double>(1.0 / client_node_config.update_frequency);
  update_timer = create_wall_timer(update_period, std::bind(&ClientNode::update_fn, this));

  RCLCPP_INFO(get_logger(), "starting publish timer.");
  std::chrono::duration<double> publish_period =
    std::chrono::duration<double>(1.0 / client_node_config.publish_frequency);
  publish_timer = create_wall_timer(publish_period, std::bind(&ClientNode::publish_fn, this));
}

void ClientNode::print_config()
{
  client_node_config.print_config();
}

void ClientNode::lift_state_callback_fn(
  const ldm_fleet_msgs::msg::LiftState::SharedPtr _msg)
{
  WriteLock ldm_state_lock(ldm_state_mutex);
  current_lift_state = *_msg;
}


messages::LiftState ClientNode::get_lift_state()
{
  messages::LiftState liftState;

  /// Checks if lift has just received a request that causes an adapter error
  if (request_error) {
    liftState.current_mode = messages::LiftState::MODE_UNKNOWN;
  } else {
    ReadLock ldm_state_lock(ldm_state_mutex);
    liftState.current_floor = current_lift_state.current_floor;
    liftState.door_state = current_lift_state.door_state;
    liftState.motion_state = current_lift_state.motion_state;
    liftState.current_mode = current_lift_state.current_mode;
    liftState.register_state = current_lift_state.register_state;
  }
  return liftState;
}

void ClientNode::publish_lift_state()
{
  messages::LiftState new_lift_state;
  new_lift_state.lift_name = client_node_config.lift_name;

  {
    ReadLock request_id_lock(request_id_mutex);
    new_lift_state.request_id = current_request_id;
  }

  ldm_fleet::messages::LiftState liftState;
  liftState = get_lift_state();

  new_lift_state.current_floor = liftState.current_floor;
  new_lift_state.door_state = liftState.door_state;
  new_lift_state.motion_state = liftState.motion_state;
  new_lift_state.current_mode = liftState.current_mode;
  new_lift_state.register_state = liftState.register_state;

  if (!fields.client->send_lift_state(new_lift_state)) {
    RCLCPP_WARN(get_logger(), "failed to send lift state");
  }
}

bool ClientNode::is_valid_request(
  const std::string & _request_lift_name,
  const std::string & _request_id)
{
  ReadLock request_id_lock(request_id_mutex);
  if (current_request_id == _request_id ||
    client_node_config.lift_name != _request_lift_name)
  {
    return false;
  }
  return true;
}

bool ClientNode::read_lift_request()
{
  messages::LiftRequest lift_request;
  if (fields.client->read_lift_request(lift_request) &&
      is_valid_request(
          lift_request.lift_name,
          lift_request.request_id))
  {
    if (((lift_request.request_type == messages::LiftRequest::REQUEST_END_SESSION) ||
         (lift_request.request_type == messages::LiftRequest::REQUEST_AGV_MODE) || 
         (lift_request.request_type == messages::LiftRequest::REQUEST_HUMAN_MODE)) && 
        ((lift_request.door_state == messages::LiftRequest::DOOR_CLOSED) ||
         (lift_request.door_state == messages::LiftRequest::DOOR_OPEN)))
    {
      if (lift_request.request_type == messages::LiftRequest::REQUEST_END_SESSION) {
        RCLCPP_INFO(get_logger(), "received REQUEST_END_SESSION command.");
      } else if (lift_request.request_type == messages::LiftRequest::REQUEST_AGV_MODE) {
        RCLCPP_INFO(get_logger(), "received REQUEST_AGV_MODE command.");
      } else {
        RCLCPP_INFO(get_logger(), "received REQUEST_HUMAN_MODE command.");
      }

      if (fields.lift_trigger_client &&
          fields.lift_trigger_client->service_is_ready())
      {
        using ServiceResponseFuture =
          rclcpp::Client<ldm_fleet_msgs::srv::Lift>::SharedFuture;
        auto response_received_callback = [&](ServiceResponseFuture future) {
          auto response = future.get();
          if (!response->success)
          {
            RCLCPP_ERROR(get_logger(), "Failed to request lift, message: %s!",
              response->message.c_str());
            request_error = true;
          } else {
            request_error = false;
          }
        };
        auto lift_srv = std::make_shared<ldm_fleet_msgs::srv::Lift::Request>();
        lift_srv->request_types = lift_request.request_type;
        lift_srv->destination_floor = lift_request.destination_floor;
        lift_srv->door_state = lift_request.door_state;

        // sync call would block indefinelty as we are in a spinning node
        fields.lift_trigger_client->async_send_request(lift_srv, response_received_callback);
      }

    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command (request_type: %d, door_state: %d).",
              lift_request.request_type, lift_request.door_state);
      request_error = true;
    }
    
    WriteLock request_id_lock(request_id_mutex);
    current_request_id = lift_request.request_id;

    return true;
  }
  return false;
}

bool ClientNode::read_register_request()
{
  messages::RegisterRequest register_request;
  if (fields.client->read_register_request(register_request) &&
      is_valid_request(
          register_request.device_name,
          register_request.request_id))
  {
    if ((register_request.register_mode == messages::RegisterRequest::REGISTER_RELEASED) ||
        (register_request.register_mode == messages::RegisterRequest::REGISTER_SIGNED)) {
      auto msg = ldm_fleet_msgs::msg::RegisterRequest();
      msg.device_name = register_request.device_name;
      msg.device_type = register_request.device_type;
      msg.register_mode = register_request.register_mode;
      if (register_request.register_mode == messages::RegisterRequest::REGISTER_RELEASED) {
        RCLCPP_INFO(get_logger(),"Publish register request: %s - RELEASED", register_request.device_name.c_str());
      } else if (register_request.register_mode == messages::RegisterRequest::REGISTER_SIGNED)
      {
        RCLCPP_INFO(get_logger(),"Publish register request: %s - SIGNED", register_request.device_name.c_str());
      }
      register_req_pub->publish(msg);
    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command (register_mode: %d).",
        register_request.register_mode);
      request_error = true;
    }

    WriteLock request_id_lock(request_id_mutex);
    current_request_id = register_request.request_id;

    return true;
  }
  return false;
}

void ClientNode::read_requests()
{
  if (read_register_request() ||
      read_lift_request())
  {
    return;
  }
}

void ClientNode::handle_requests()
{
}

void ClientNode::update_fn()
{
  read_requests();
}

void ClientNode::publish_fn()
{
  publish_lift_state();
}

} // namespace ros2
} // namespace ldm_fleet
