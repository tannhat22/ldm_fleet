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

#ifndef LDM_FLEET__INCLUDE__LDM_FLEET__CLIENTCONFIG_HPP
#define LDM_FLEET__INCLUDE__LDM_FLEET__CLIENTCONFIG_HPP

#include <string>

namespace ldm_fleet {

struct ClientConfig
{
  int dds_domain = 52;
  std::string dds_state_topic = "lift_state";

  // Client request -> server:

  // Server request -> client:
  std::string dds_lift_request_topic = "lift_request";
  std::string dds_register_request_topic = "register_request";

  void print_config() const;
};

} // namespace ldm_fleet

#endif // LDM_FLEET__INCLUDE__LDM_FLEET__CLIENTCONFIG_HPP
