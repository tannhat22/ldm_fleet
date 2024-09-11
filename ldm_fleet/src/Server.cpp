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

#include <ldm_fleet/Server.hpp>

#include "ServerImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace ldm_fleet {

Server::SharedPtr Server::make(const ServerConfig& _config)
{
  SharedPtr server = SharedPtr(new Server(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSSubscribeHandler<LdmFleetData_LiftState, 10>::SharedPtr state_sub(
      new dds::DDSSubscribeHandler<LdmFleetData_LiftState, 10>(
          participant, &LdmFleetData_LiftState_desc,
          _config.dds_state_topic));

  dds::DDSPublishHandler<LdmFleetData_LiftRequest>::SharedPtr 
      lift_request_pub(
          new dds::DDSPublishHandler<LdmFleetData_LiftRequest>(
              participant, &LdmFleetData_LiftRequest_desc,
              _config.dds_lift_request_topic));

  dds::DDSPublishHandler<LdmFleetData_RegisterRequest>::SharedPtr 
      register_request_pub(
          new dds::DDSPublishHandler<LdmFleetData_RegisterRequest>(
              participant, &LdmFleetData_RegisterRequest_desc,
              _config.dds_register_request_topic));

  if (!state_sub->is_ready() ||
      !lift_request_pub->is_ready() ||
      !register_request_pub->is_ready())
    return nullptr;

  server->impl->start(ServerImpl::Fields{
      std::move(participant),
      std::move(state_sub),
      std::move(lift_request_pub),
      std::move(register_request_pub)});
  return server;
}

Server::Server(const ServerConfig& _config)
{
  impl.reset(new ServerImpl(_config));
}

Server::~Server()
{}

bool Server::read_lift_states(
    std::vector<messages::LiftState>& _new_lift_states)
{
  return impl->read_lift_states(_new_lift_states);
}

bool Server::send_lift_request(const messages::LiftRequest& _lift_request)
{
  return impl->send_lift_request(_lift_request);
}

bool Server::send_register_request(const messages::RegisterRequest& _register_request)
{
  return impl->send_register_request(_register_request);
}

} // namespace ldm_fleet
