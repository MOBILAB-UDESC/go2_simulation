// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/AudioData.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/audio_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__AudioData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x20, 0x63, 0x91, 0x5f, 0x92, 0xdb, 0x5c, 0x13,
      0xca, 0x90, 0x23, 0x05, 0x58, 0xf3, 0x0c, 0xe2,
      0x13, 0xc3, 0xd1, 0xe6, 0x33, 0x2b, 0xc8, 0xe0,
      0x8a, 0x82, 0x42, 0xd9, 0xaf, 0x02, 0x1c, 0xa5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__AudioData__TYPE_NAME[] = "go2_interfaces/msg/AudioData";

// Define type names, field names, and default values
static char go2_interfaces__msg__AudioData__FIELD_NAME__time_frame[] = "time_frame";
static char go2_interfaces__msg__AudioData__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__AudioData__FIELDS[] = {
  {
    {go2_interfaces__msg__AudioData__FIELD_NAME__time_frame, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__AudioData__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__AudioData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__AudioData__TYPE_NAME, 28, 28},
      {go2_interfaces__msg__AudioData__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 time_frame\n"
  "uint8[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__AudioData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__AudioData__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 30, 30},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__AudioData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__AudioData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
