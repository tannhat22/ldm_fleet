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

#include <ldm_fleet/ServerConfig.hpp>

#include <cstdio>

namespace ldm_fleet {

void ServerConfig::print_config() const
{
  printf("SERVER-CLIENT DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    lift state: %s\n", dds_state_topic.c_str());
  printf("    lift request: %s\n", dds_lift_request_topic.c_str());
  printf("    register request: %s\n", dds_register_request_topic.c_str());
}

} // namespace ldm_fleet
