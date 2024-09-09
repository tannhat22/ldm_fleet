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

#ifndef LDM_FLEET__INCLUDE__LDM_FLEET__CLIENT_HPP
#define LDM_FLEET__INCLUDE__LDM_FLEET__CLIENT_HPP

#include <memory>

#include <ldm_fleet/ClientConfig.hpp>

#include <ldm_fleet/messages/LiftState.hpp>
#include <ldm_fleet/messages/LiftRequest.hpp>

namespace ldm_fleet {

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;

  /// Factory function that creates an instance of the Ldm Fleet DDS Client.
  ///
  /// \param[in] config
  ///   Configuration that sets up the client to communicate with the server.
  /// \return
  ///   Shared pointer to a ldm fleet client.
  static SharedPtr make(const ClientConfig& config);

  /// Attempts to send a new lift state to the ldm fleet server, to be 
  /// registered by the fleet management system.
  ///
  /// \param[in] new_lift_state
  ///   Current lift state to be sent to the ldm fleet server to update the
  ///   fleet management system.
  /// \return
  ///   True if lift state was successfully sent, false otherwise.
  bool send_lift_state(const messages::LiftState& new_lift_state);

  /// Attempts to read and receive a new ldm request from the ldm fleet
  /// server, for commanding the ldm client.
  ///
  /// \param[out] lift_request
  ///   Newly received lift request from the ldm fleet server, to be
  ///   handled by the ldm client.
  /// \return
  ///   True if a new lift request was received, false otherwise.
  bool read_lift_request(messages::LiftRequest& lift_request);

  /// Destructor
  ~Client();

private:

  /// Forward declaration and unique implementation
  class ClientImpl;

  std::unique_ptr<ClientImpl> impl;

  Client(const ClientConfig& config);

};

} // namespace ldm_fleet

#endif // LDM_FLEET__INCLUDE__LDM_FLEET__CLIENT_HPP
