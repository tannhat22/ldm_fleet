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

#include "ServerImpl.hpp"
#include "messages/message_utils.hpp"

namespace ldm_fleet {

Server::ServerImpl::ServerImpl(const ServerConfig& _config) :
  server_config(_config)
{}

Server::ServerImpl::~ServerImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Server::ServerImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Server::ServerImpl::read_lift_states(
    std::vector<messages::LiftState>& _new_lift_states)
{
  auto lift_states = fields.state_sub->read();
  if (!lift_states.empty())
  {
    _new_lift_states.clear();
    for (size_t i = 0; i < lift_states.size(); ++i)
    {
      messages::LiftState tmp_lift_state;
      convert(*(lift_states[i]), tmp_lift_state);
      _new_lift_states.push_back(tmp_lift_state);
    }
    return true;
  }
  return false;
}

bool Server::ServerImpl::send_lift_request(
    const messages::LiftRequest& _lift_request)
{
  LdmFleetData_LiftRequest* new_mr = LdmFleetData_LiftRequest__alloc();
  convert(_lift_request, *new_mr);
  bool sent = fields.lift_request_pub->write(new_mr);
  LdmFleetData_LiftRequest_free(new_mr, DDS_FREE_ALL);
  return sent;
}

} // namespace ldm_fleet
