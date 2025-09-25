// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/MotorCmds.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/motor_cmds__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__MotorCmds__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x37, 0xf6, 0xff, 0x8b, 0x2e, 0xa7, 0x57, 0x17,
      0x2d, 0xc1, 0xbc, 0xe4, 0x28, 0x89, 0x9a, 0x8e,
      0x67, 0xe1, 0xb3, 0xe0, 0x3c, 0xad, 0xff, 0xb4,
      0x22, 0x2c, 0x77, 0x5d, 0x68, 0x20, 0xca, 0x95,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "go2_interfaces/msg/detail/motor_cmd__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t go2_interfaces__msg__MotorCmd__EXPECTED_HASH = {1, {
    0xa8, 0xa7, 0x7b, 0x38, 0xdc, 0x13, 0xfc, 0x60,
    0xcb, 0xa8, 0x46, 0xe8, 0x98, 0x12, 0xd8, 0xd7,
    0xe6, 0x75, 0x14, 0xd6, 0x68, 0x31, 0x6a, 0x54,
    0xc1, 0xd6, 0xa8, 0x04, 0x9d, 0x29, 0xde, 0x6e,
  }};
#endif

static char go2_interfaces__msg__MotorCmds__TYPE_NAME[] = "go2_interfaces/msg/MotorCmds";
static char go2_interfaces__msg__MotorCmd__TYPE_NAME[] = "go2_interfaces/msg/MotorCmd";

// Define type names, field names, and default values
static char go2_interfaces__msg__MotorCmds__FIELD_NAME__cmds[] = "cmds";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__MotorCmds__FIELDS[] = {
  {
    {go2_interfaces__msg__MotorCmds__FIELD_NAME__cmds, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {go2_interfaces__msg__MotorCmd__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription go2_interfaces__msg__MotorCmds__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {go2_interfaces__msg__MotorCmd__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__MotorCmds__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__MotorCmds__TYPE_NAME, 28, 28},
      {go2_interfaces__msg__MotorCmds__FIELDS, 1, 1},
    },
    {go2_interfaces__msg__MotorCmds__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&go2_interfaces__msg__MotorCmd__EXPECTED_HASH, go2_interfaces__msg__MotorCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = go2_interfaces__msg__MotorCmd__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "MotorCmd[] cmds";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__MotorCmds__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__MotorCmds__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 15, 15},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__MotorCmds__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__MotorCmds__get_individual_type_description_source(NULL),
    sources[1] = *go2_interfaces__msg__MotorCmd__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
