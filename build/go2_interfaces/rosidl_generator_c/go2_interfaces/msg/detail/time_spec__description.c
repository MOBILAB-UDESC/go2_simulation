// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/TimeSpec.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/time_spec__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__TimeSpec__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa8, 0x8c, 0xea, 0x87, 0xa0, 0x24, 0x0f, 0x05,
      0xe2, 0x4a, 0xf1, 0x1f, 0xd7, 0x9b, 0x2d, 0xf9,
      0x74, 0x73, 0xc6, 0x30, 0x88, 0x1a, 0x44, 0x9d,
      0x3f, 0xbc, 0x8b, 0x8c, 0x6d, 0x63, 0xbb, 0x86,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__TimeSpec__TYPE_NAME[] = "go2_interfaces/msg/TimeSpec";

// Define type names, field names, and default values
static char go2_interfaces__msg__TimeSpec__FIELD_NAME__sec[] = "sec";
static char go2_interfaces__msg__TimeSpec__FIELD_NAME__nanosec[] = "nanosec";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__TimeSpec__FIELDS[] = {
  {
    {go2_interfaces__msg__TimeSpec__FIELD_NAME__sec, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__TimeSpec__FIELD_NAME__nanosec, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__TimeSpec__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__TimeSpec__TYPE_NAME, 27, 27},
      {go2_interfaces__msg__TimeSpec__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Time indicates a specific point in time, relative to a clock's 0 point.\n"
  "# The seconds component, valid over all int32 values.\n"
  "int32 sec\n"
  "# The nanoseconds component, valid in the range [0, 10e9).\n"
  "uint32 nanosec";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__TimeSpec__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__TimeSpec__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 211, 211},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__TimeSpec__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__TimeSpec__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
