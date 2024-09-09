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

#ifndef LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__LDMSTATE_HPP
#define LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__LDMSTATE_HPP

#include <string>
#include <vector>

namespace ldm_fleet {
namespace messages {

struct LiftState
{
  std::string lift_name;
  std::string current_floor;

  uint32_t door_state;
  static const uint32_t DOOR_CLOSED = 0;
  static const uint32_t DOOR_MOVING = 1;
  static const uint32_t DOOR_OPEN = 2;

  uint32_t motion_state;
  static const uint32_t MOTION_STOPPED = 0;
  static const uint32_t MOTION_UP = 1;
  static const uint32_t MOTION_DOWN = 2;
  static const uint32_t MOTION_UNKNOWN = 3;

  uint32_t current_mode;
  static const uint32_t MODE_UNKNOWN = 0;
  static const uint32_t MODE_HUMAN = 1;
  static const uint32_t MODE_AGV = 2;
  static const uint32_t MODE_FIRE = 3;
  static const uint32_t MODE_OFFLINE = 4;
  static const uint32_t MODE_EMERGENCY = 5;

  std::string request_id;

};

} // namespace messages
} // namespace ldm_fleet

#endif // LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__LDMSTATE_HPP
