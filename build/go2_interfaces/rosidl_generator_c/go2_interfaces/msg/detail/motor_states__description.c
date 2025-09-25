// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/MotorStates.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/motor_states__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__MotorStates__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x14, 0x76, 0xbd, 0xe2, 0x1a, 0x73, 0x92, 0xf6,
      0x4d, 0xa3, 0x03, 0x7f, 0x20, 0x6b, 0x93, 0x28,
      0x09, 0xc4, 0xf6, 0x31, 0x99, 0x2f, 0x7e, 0xb7,
      0x9c, 0xda, 0x2f, 0xee, 0x4b, 0x9b, 0xf3, 0x64,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "go2_interfaces/msg/detail/motor_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t go2_interfaces__msg__MotorState__EXPECTED_HASH = {1, {
    0x7a, 0x42, 0xfb, 0xae, 0x9f, 0xad, 0xf7, 0x51,
    0xed, 0xda, 0xb2, 0x43, 0xd1, 0xa9, 0xab, 0xb5,
    0x4c, 0xed, 0xd4, 0x23, 0x9e, 0x43, 0xa1, 0x75,
    0x17, 0xb6, 0xba, 0x76, 0xc1, 0x08, 0x14, 0x3a,
  }};
#endif

static char go2_interfaces__msg__MotorStates__TYPE_NAME[] = "go2_interfaces/msg/MotorStates";
static char go2_interfaces__msg__MotorState__TYPE_NAME[] = "go2_interfaces/msg/MotorState";

// Define type names, field names, and default values
static char go2_interfaces__msg__MotorStates__FIELD_NAME__states[] = "states";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__MotorStates__FIELDS[] = {
  {
    {go2_interfaces__msg__MotorStates__FIELD_NAME__states, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {go2_interfaces__msg__MotorState__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription go2_interfaces__msg__MotorStates__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {go2_interfaces__msg__MotorState__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__MotorStates__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__MotorStates__TYPE_NAME, 30, 30},
      {go2_interfaces__msg__MotorStates__FIELDS, 1, 1},
    },
    {go2_interfaces__msg__MotorStates__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&go2_interfaces__msg__MotorState__EXPECTED_HASH, go2_interfaces__msg__MotorState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = go2_interfaces__msg__MotorState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "MotorState[] states";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__MotorStates__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__MotorStates__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 19, 19},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__MotorStates__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__MotorStates__get_individual_type_description_source(NULL),
    sources[1] = *go2_interfaces__msg__MotorState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
