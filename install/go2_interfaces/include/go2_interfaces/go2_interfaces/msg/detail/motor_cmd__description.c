// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/MotorCmd.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/motor_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__MotorCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa8, 0xa7, 0x7b, 0x38, 0xdc, 0x13, 0xfc, 0x60,
      0xcb, 0xa8, 0x46, 0xe8, 0x98, 0x12, 0xd8, 0xd7,
      0xe6, 0x75, 0x14, 0xd6, 0x68, 0x31, 0x6a, 0x54,
      0xc1, 0xd6, 0xa8, 0x04, 0x9d, 0x29, 0xde, 0x6e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__MotorCmd__TYPE_NAME[] = "go2_interfaces/msg/MotorCmd";

// Define type names, field names, and default values
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__mode[] = "mode";
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__q[] = "q";
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__dq[] = "dq";
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__tau[] = "tau";
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__kp[] = "kp";
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__kd[] = "kd";
static char go2_interfaces__msg__MotorCmd__FIELD_NAME__reserve[] = "reserve";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__MotorCmd__FIELDS[] = {
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__dq, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__tau, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__kp, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__FIELD_NAME__reserve, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__MotorCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__MotorCmd__TYPE_NAME, 27, 27},
      {go2_interfaces__msg__MotorCmd__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 mode\n"
  "float32 q\n"
  "float32 dq\n"
  "float32 tau\n"
  "float32 kp\n"
  "float32 kd\n"
  "uint32[3] reserve";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__MotorCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__MotorCmd__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 83, 83},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__MotorCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__MotorCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
