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

#ifndef LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__REGISTERREQUEST_HPP
#define LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__REGISTERREQUEST_HPP

#include <string>

namespace ldm_fleet {
namespace messages {

struct RegisterRequest
{
  std::string request_id;
  std::string device_name;

  uint32_t device_type;
  static const uint32_t DEVICE_DOOR = 0;
  static const uint32_t DEVICE_LIFT = 1;

  uint32_t register_mode;
  static const uint32_t REGISTER_RELEASED = 0;
  static const uint32_t REGISTER_SIGNED = 1;
};

} // namespace messages
} // namespace ldm_fleet

#endif // LDM_FLEET__INCLUDE__LDM_FLEET__MESSAGES__REGISTERREQUEST_HPP
