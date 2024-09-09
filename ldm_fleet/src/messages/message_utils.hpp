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

#ifndef LDM_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP
#define LDM_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP

#include <ldm_fleet/messages/LiftRequest.hpp>
#include <ldm_fleet/messages/LiftState.hpp>

#include "FleetMessages.h"

namespace ldm_fleet {
namespace messages {

void convert(const LiftRequest& _input, LdmFleetData_LiftRequest& _output);

void convert(const LdmFleetData_LiftRequest& _input, LiftRequest& _output);

void convert(const LiftState& _input, LdmFleetData_LiftState& _output);

void convert(const LdmFleetData_LiftState& _input, LiftState& _output);

} // namespace 
} // namespace ldm_fleet

#endif // LDM_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP
