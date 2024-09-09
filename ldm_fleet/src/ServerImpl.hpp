/*
 * Copyright (C) 2019 Open Source Machineics Foundation
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

#ifndef LDM_FLEET__SRC__SERVERIMPL_HPP
#define LDM_FLEET__SRC__SERVERIMPL_HPP

#include <ldm_fleet/messages/LiftRequest.hpp>
#include <ldm_fleet/messages/LiftState.hpp>
#include <ldm_fleet/Server.hpp>
#include <ldm_fleet/ServerConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace ldm_fleet {

class Server::ServerImpl
{
public:

  /// DDS related fields required for the server to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS subscribers for new incoming lift states from clients
    dds::DDSSubscribeHandler<LdmFleetData_LiftState, 10>::SharedPtr
        state_sub;

    /// DDS publisher for lift requests to be sent to clients
    dds::DDSPublishHandler<LdmFleetData_LiftRequest>::SharedPtr
        lift_request_pub;
  };

  ServerImpl(const ServerConfig& config);

  ~ServerImpl();

  void start(Fields fields);

  bool read_lift_states(std::vector<messages::LiftState>& new_lift_states);

  bool send_lift_request(const messages::LiftRequest& lift_request);

private:

  Fields fields;

  ServerConfig server_config;

};

} // namespace ldm_fleet

#endif // LDM_FLEET__SRC__SERVERIMPL_HPP
