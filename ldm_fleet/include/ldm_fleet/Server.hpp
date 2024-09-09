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

#ifndef LDM_FLEET__INCLUDE__LDM_FLEET__SERVER_HPP
#define LDM_FLEET__INCLUDE__LDM_FLEET__SERVER_HPP

#include <memory>
#include <vector>

#include <ldm_fleet/ServerConfig.hpp>

#include <ldm_fleet/messages/LiftState.hpp>
#include <ldm_fleet/messages/LiftRequest.hpp>

namespace ldm_fleet {

class Server
{
public:

  using SharedPtr = std::shared_ptr<Server>;

  /// Factory function that creates an instance of the Ldm Fleet Server.
  ///
  /// \param[in] config
  ///   Configuration that sets up the server to communicate with the clients.
  /// \return
  ///   Shared pointer to a ldm fleet server.
  static SharedPtr make(const ServerConfig& config);

  /// Attempts to read new incoming lift states sent by ldm fleet clients
  /// over DDS.
  ///
  /// \param[out] new_lift_states
  ///   A vector of new incoming lift states sent by clients to update the
  ///   fleet management system.
  /// \return
  ///   True if new lift states were received, false otherwise.
  bool read_lift_states(std::vector<messages::LiftState>& new_lift_states);

  /// Attempts to send a new lift request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  /// 
  /// \param[in] lift_request
  ///   New lift request to be sent out to the clients.
  /// \return
  ///   True if the lift request was successfully sent, false otherwise.
  bool send_lift_request(const messages::LiftRequest& lift_request);

  /// Destructor
  ~Server();

private:

  /// Forward declaration and unique implementation
  class ServerImpl;

  std::unique_ptr<ServerImpl> impl;

  Server(const ServerConfig& config);

};

} // namespace ldm_fleet

#endif // LDM_FLEET__INCLUDE__LDM_FLEET__SERVER_HPP
