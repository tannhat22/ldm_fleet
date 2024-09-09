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

#include <dds/dds.h>

#include "../messages/FleetMessages.h"

int main(int argc, char** argv)
{
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t reader;

  LdmFleetData_LiftState* msg;
  void* samples[1];
  dds_sample_info_t infos[1];

  dds_return_t rc;
  dds_qos_t* qos;

  (void)argc;
  (void)argv;

  /* Create a participant */
  participant = dds_create_participant(52, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &LdmFleetData_LiftState_desc, 
    "lift_state", NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a best effort Reader (UDP) */
  qos = dds_create_qos ();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  reader = dds_create_reader (participant, topic, qos, NULL);
  if (reader < 0)
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  dds_delete_qos(qos);

  printf ("\n=== [Subscriber] Waiting for a sample ...\n");
  fflush (stdout);

  /* Initialize sample buffer, by pointing the void pointer within
   * the buffer array to a valid sample memory location. */
  samples[0] = LdmFleetData_LiftState__alloc();

  /* Poll until data has been read. */
  while (true)
  {
    /* Do the actual read.
     * The return value contains the number of read samples. */
    rc = dds_take(reader, samples, infos, 1, 1);
    if (rc < 0)
      DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));

    /* Check if we read some data and it is valid. */
    if ((rc > 0) && (infos[0].valid_data))
    {
      /* Print Message. */
      msg = (LdmFleetData_LiftState*)samples[0];
      std::cout << "=== [Subscriber] Received : " << std::endl;
      std::cout << "lift_name: " << msg->lift_name << std::endl;
      std::cout << "current_floor: " << msg->current_floor << std::endl;
      std::cout << "request_id: " << msg->request_id << std::endl;
      
      std::cout << "door_state: ";
      if (msg->door_state == LdmFleetData_LiftState_Constants_DOOR_CLOSED)
        std::cout << "CLOSED" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_DOOR_MOVING)
        std::cout << "MOVING" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_DOOR_OPEN)
        std::cout << "OPEN" << std::endl;

      std::cout << "motion_state: ";
      if (msg->door_state == LdmFleetData_LiftState_Constants_MOTION_STOPPED)
        std::cout << "STOPPED" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MOTION_UP)
        std::cout << "UP" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MOTION_DOWN)
        std::cout << "DOWN" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MOTION_UNKNOWN)
        std::cout << "UNKNOWN" << std::endl;   

      std::cout << "current_mode: ";
      if (msg->door_state == LdmFleetData_LiftState_Constants_MODE_UNKNOWN)
        std::cout << "UNKNOWN" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MODE_HUMAN)
        std::cout << "HUMAN" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MODE_AGV)
        std::cout << "AGV" << std::endl;
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MODE_FIRE)
        std::cout << "FIRE" << std::endl;       
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MODE_OFFLINE)
        std::cout << "OFFLINE" << std::endl;  
      else if (msg->door_state == LdmFleetData_LiftState_Constants_MODE_EMERGENCY)
        std::cout << "EMERGENCY" << std::endl;           
      //break;
    }
    else
    {
      /* Polling sleep. */
      dds_sleepfor (DDS_MSECS (20));
    }
  }

  /* Machine the data location. */
  LdmFleetData_LiftState_free (samples[0], DDS_FREE_ALL);

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return EXIT_SUCCESS;
}
