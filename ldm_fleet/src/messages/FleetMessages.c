/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: FleetMessages.c
  Source: FleetMessages.idl
  Cyclone DDS: V0.7.0

*****************************************************************/
#include "FleetMessages.h"


static const uint32_t LdmFleetData_LiftRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_LiftRequest, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_LiftRequest, lift_name),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_LiftRequest, request_type),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_LiftRequest, destination_floor),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_LiftRequest, door_state),
  DDS_OP_RTS
};

const dds_topic_descriptor_t LdmFleetData_LiftRequest_desc =
{
  sizeof (LdmFleetData_LiftRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "LdmFleetData::LiftRequest",
  NULL,
  6,
  LdmFleetData_LiftRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"LdmFleetData\"><Struct name=\"LiftRequest\"><Member name=\"request_id\"><String/></Member><Member name=\"lift_name\"><String/></Member><Member name=\"request_type\"><ULong/></Member><Member name=\"destination_floor\"><String/></Member><Member name=\"door_state\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t LdmFleetData_LiftState_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_LiftState, lift_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_LiftState, current_floor),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_LiftState, door_state),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_LiftState, motion_state),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_LiftState, current_mode),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_LiftState, register_state),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_LiftState, request_id),
  DDS_OP_RTS
};

const dds_topic_descriptor_t LdmFleetData_LiftState_desc =
{
  sizeof (LdmFleetData_LiftState),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "LdmFleetData::LiftState",
  NULL,
  8,
  LdmFleetData_LiftState_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"LdmFleetData\"><Struct name=\"LiftState\"><Member name=\"lift_name\"><String/></Member><Member name=\"current_floor\"><String/></Member><Member name=\"door_state\"><ULong/></Member><Member name=\"motion_state\"><ULong/></Member><Member name=\"current_mode\"><ULong/></Member><Member name=\"register_state\"><ULong/></Member><Member name=\"request_id\"><String/></Member></Struct></Module></MetaData>"
};


static const uint32_t LdmFleetData_RegisterRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_RegisterRequest, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (LdmFleetData_RegisterRequest, device_name),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_RegisterRequest, device_type),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (LdmFleetData_RegisterRequest, register_mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t LdmFleetData_RegisterRequest_desc =
{
  sizeof (LdmFleetData_RegisterRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "LdmFleetData::RegisterRequest",
  NULL,
  5,
  LdmFleetData_RegisterRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"LdmFleetData\"><Struct name=\"RegisterRequest\"><Member name=\"request_id\"><String/></Member><Member name=\"device_name\"><String/></Member><Member name=\"device_type\"><ULong/></Member><Member name=\"register_mode\"><ULong/></Member></Struct></Module></MetaData>"
};
