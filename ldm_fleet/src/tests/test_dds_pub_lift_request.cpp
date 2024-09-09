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
    std::cout << "<Executable> <Lift name> <Request type> <Destination floor> <Door state> <Request id>" << std::endl;
    return 1;
  }

  std::string lift_name(argv[0]);
  std::string request_type(argv[1]);
  std::string destination_floor(argv[2]);
  std::string door_state(argv[3]);
  std::string request_id(argv[4]);

  if (request_type != "end" && 
      request_type != "agv" &&
      request_type != "human")
  {
    std::cout << "Supported lift request_type are End_session, Agv_mode and Human_mode." << std::endl;
    return 1;
  }

  if (door_state != "closed" && 
      door_state != "open")
  {
    std::cout << "Supported lift door_state are closed and open." << std::endl;
    return 1;
  }

  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  dds_qos_t *qos;
  LdmFleetData_LiftRequest* msg;
  msg = LdmFleetData_LiftRequest__alloc();
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  /* Create a Participant. */
  participant = dds_create_participant(52, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &LdmFleetData_LiftRequest_desc, "lift_request", 
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
  msg->lift_name = ldm_fleet::common::dds_string_alloc_and_copy(lift_name);
  msg->destination_floor = ldm_fleet::common::dds_string_alloc_and_copy(destination_floor);

  if (request_type == "end")
    msg->request_type = LdmFleetData_LiftRequest_Constants_REQUEST_END_SESSION;
  else if (request_type == "agv")
    msg->request_type = LdmFleetData_LiftRequest_Constants_REQUEST_AGV_MODE;
  else if (request_type == "human")
    msg->request_type = LdmFleetData_LiftRequest_Constants_REQUEST_HUMAN_MODE;
  
  printf ("=== [Publisher]  Writing : ");
  printf ("Message: request_type %s\n", request_type.c_str());
  printf ("Message: destination_floor %s\n", destination_floor.c_str());
  printf ("Message: door_state %s\n", door_state.c_str());
  fflush (stdout);

  rc = dds_write (writer, msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  LdmFleetData_LiftRequest_free(msg, DDS_FREE_ALL);

  return EXIT_SUCCESS;
}
