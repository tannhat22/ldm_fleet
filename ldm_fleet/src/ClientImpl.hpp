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

#ifndef LDM_FLEET__SRC__CLIENTIMPL_HPP
#define LDM_FLEET__SRC__CLIENTIMPL_HPP

#include <ldm_fleet/messages/LiftRequest.hpp>
#include <ldm_fleet/messages/LiftState.hpp>
#include <ldm_fleet/messages/RegisterRequest.hpp>
#include <ldm_fleet/Client.hpp>
#include <ldm_fleet/ClientConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace ldm_fleet {

class Client::ClientImpl
{
public:

  /// DDS related fields required for the client to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS publisher that handles sending out current lift states to the 
    /// server
    dds::DDSPublishHandler<LdmFleetData_LiftState>::SharedPtr
        state_pub;

    /// DDS subscriber for lift requests coming from the server
    dds::DDSSubscribeHandler<LdmFleetData_LiftRequest>::SharedPtr 
        lift_request_sub;

    /// DDS subscriber for register requests coming from the server
    dds::DDSSubscribeHandler<LdmFleetData_RegisterRequest>::SharedPtr 
        register_request_sub;
  };

  ClientImpl(const ClientConfig& config);

  ~ClientImpl();

  void start(Fields fields);

  bool send_lift_state(const messages::LiftState& new_lift_state);

  bool read_lift_request(messages::LiftRequest& lift_request);

  bool read_register_request(messages::RegisterRequest& register_request);

private:

  Fields fields;

  ClientConfig client_config;

};

} // namespace ldm_fleet

#endif // LDM_FLEET__SRC__CLIENTIMPL_HPP
