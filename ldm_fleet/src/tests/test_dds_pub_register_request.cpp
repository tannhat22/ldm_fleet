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

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include <iostream>

#include <dds/dds.h>

#include "../messages/FleetMessages.h"
#include "../dds_utils/common.hpp"

int main (int argc, char ** argv)
{
  if (argc < 4)
  {
    std::cout << "Please request using the following format," << std::endl;
    std::cout << "<Executable> <Device name> <Device type> <Register mode> <Request id>" << std::endl;
    return 1;
  }

  std::string device_name(argv[0]);
  std::string device_type(argv[1]);
  std::string register_mode(argv[2]);
  std::string request_id(argv[3]);

  if (device_type != "door" && 
      device_type != "lift")
  {
    std::cout << "Supported register device_type are door and lift." << std::endl;
    return 1;
  }

  if (register_mode != "released" && 
      register_mode != "signed")
  {
    std::cout << "Supported register mode are released and signed." << std::endl;
    return 1;
  }

  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  dds_qos_t *qos;
  LdmFleetData_RegisterRequest* msg;
  msg = LdmFleetData_RegisterRequest__alloc();
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  /* Create a Participant. */
  participant = dds_create_participant(52, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &LdmFleetData_RegisterRequest_desc, "register_request", 
    NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a Writer. */
  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  writer = dds_create_writer (participant, topic, qos, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_write: %s\n", dds_strretcode(-writer));
  dds_delete_qos(qos);

  printf("=== [Publisher]  Waiting for a reader to be discovered ...\n");
  fflush (stdout);

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
  {
    rc = dds_get_status_changes (writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor (DDS_MSECS (20));
  }

  /* Create a message to write. */
  msg->request_id = ldm_fleet::common::dds_string_alloc_and_copy(request_id);
  msg->device_name = ldm_fleet::common::dds_string_alloc_and_copy(device_name);

  if (device_type == "door")
    msg->device_type = LdmFleetData_RegisterRequest_Constants_DEVICE_DOOR;
  else if (device_type == "lift")
    msg->device_type = LdmFleetData_RegisterRequest_Constants_DEVICE_LIFT;
  
  if (register_mode == "released")
    msg->register_mode = LdmFleetData_RegisterRequest_Constants_REGISTER_RELEASED;
  else if (register_mode == "signed")
    msg->register_mode = LdmFleetData_RegisterRequest_Constants_REGISTER_SIGNED;

  printf ("=== [Publisher]  Writing : ");
  printf ("Message: device_name %s\n", device_name.c_str());
  printf ("Message: device_type %s\n", device_type.c_str());
  printf ("Message: register_mode %s\n", register_mode.c_str());
  fflush (stdout);

  rc = dds_write (writer, msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  LdmFleetData_RegisterRequest_free(msg, DDS_FREE_ALL);

  return EXIT_SUCCESS;
}
