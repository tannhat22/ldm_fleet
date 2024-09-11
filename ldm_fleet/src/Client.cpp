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

#include <ldm_fleet/Client.hpp>

#include "ClientImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace ldm_fleet {

Client::SharedPtr Client::make(const ClientConfig& _config)
{
  SharedPtr client = SharedPtr(new Client(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSPublishHandler<LdmFleetData_LiftState>::SharedPtr state_pub(
      new dds::DDSPublishHandler<LdmFleetData_LiftState>(
          participant, &LdmFleetData_LiftState_desc,
          _config.dds_state_topic));

  dds::DDSSubscribeHandler<LdmFleetData_LiftRequest>::SharedPtr 
      lift_request_sub(
          new dds::DDSSubscribeHandler<LdmFleetData_LiftRequest>(
              participant, &LdmFleetData_LiftRequest_desc,
              _config.dds_lift_request_topic));

  dds::DDSSubscribeHandler<LdmFleetData_RegisterRequest>::SharedPtr 
      register_request_sub(
          new dds::DDSSubscribeHandler<LdmFleetData_RegisterRequest>(
              participant, &LdmFleetData_RegisterRequest_desc,
              _config.dds_register_request_topic));

  if (!state_pub->is_ready() ||
      !lift_request_sub->is_ready() ||
      !register_request_sub->is_ready())
    return nullptr;

  client->impl->start(ClientImpl::Fields{
      std::move(participant),
      std::move(state_pub),
      std::move(lift_request_sub),
      std::move(register_request_sub)});
      
  return client;
}

Client::Client(const ClientConfig& _config)
{
  impl.reset(new ClientImpl(_config));
}

Client::~Client()
{}

bool Client::send_lift_state(const messages::LiftState& _new_lift_state)
{
  return impl->send_lift_state(_new_lift_state);
}

bool Client::read_lift_request(messages::LiftRequest& _lift_request)
{
  return impl->read_lift_request(_lift_request);
}

bool Client::read_register_request(messages::RegisterRequest& _register_request)
{
  return impl->read_register_request(_register_request);
}

} // namespace ldm_fleet
