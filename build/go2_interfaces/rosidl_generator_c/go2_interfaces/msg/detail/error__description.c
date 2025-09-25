// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/Error.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/error__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__Error__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x83, 0xc4, 0xa6, 0xa2, 0x52, 0x9b, 0xe8, 0x63,
      0x50, 0xac, 0x43, 0x71, 0x7b, 0x2d, 0x53, 0x53,
      0x9b, 0x98, 0x22, 0x1b, 0xd2, 0x4b, 0x87, 0xde,
      0x65, 0x10, 0x1c, 0xd4, 0x48, 0x02, 0xa7, 0x64,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__Error__TYPE_NAME[] = "go2_interfaces/msg/Error";

// Define type names, field names, and default values
static char go2_interfaces__msg__Error__FIELD_NAME__source[] = "source";
static char go2_interfaces__msg__Error__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__Error__FIELDS[] = {
  {
    {go2_interfaces__msg__Error__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Error__FIELD_NAME__state, 5, 5},
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
go2_interfaces__msg__Error__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__Error__TYPE_NAME, 24, 24},
      {go2_interfaces__msg__Error__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 source\n"
  "uint32 state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__Error__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__Error__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__Error__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__Error__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
