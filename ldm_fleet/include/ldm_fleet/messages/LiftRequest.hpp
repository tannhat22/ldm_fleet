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

#ifndef LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__LDMREQUEST_HPP
#define LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__LDMREQUEST_HPP

#include <string>

namespace ldm_fleet {
namespace messages {

struct LiftRequest
{
  std::string request_id;
  std::string lift_name;

  uint32_t request_type;
  static const uint32_t REQUEST_END_SESSION = 0;
  static const uint32_t REQUEST_AGV_MODE = 1;
  static const uint32_t REQUEST_HUMAN_MODE = 2;

  std::string destination_floor;

  uint32_t door_state;
  static const uint32_t DOOR_CLOSED = 0;
  static const uint32_t DOOR_OPEN = 2;  
};

} // namespace messages
} // namespace ldm_fleet

#endif // LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__LDMREQUEST_HPP
