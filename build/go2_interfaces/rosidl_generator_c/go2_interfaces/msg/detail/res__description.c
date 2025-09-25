// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/Res.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/res__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__Res__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1c, 0x55, 0x86, 0xbe, 0x20, 0x8f, 0x44, 0x3d,
      0x27, 0x1f, 0x4f, 0x5f, 0x22, 0xe6, 0xc6, 0xbc,
      0xee, 0xf0, 0xe9, 0xa2, 0xae, 0x27, 0x2b, 0x53,
      0xd9, 0xa3, 0xc6, 0x90, 0xc3, 0x78, 0xc7, 0x33,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__Res__TYPE_NAME[] = "go2_interfaces/msg/Res";

// Define type names, field names, and default values
static char go2_interfaces__msg__Res__FIELD_NAME__uuid[] = "uuid";
static char go2_interfaces__msg__Res__FIELD_NAME__data[] = "data";
static char go2_interfaces__msg__Res__FIELD_NAME__body[] = "body";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__Res__FIELDS[] = {
  {
    {go2_interfaces__msg__Res__FIELD_NAME__uuid, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Res__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Res__FIELD_NAME__body, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__Res__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__Res__TYPE_NAME, 22, 22},
      {go2_interfaces__msg__Res__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string uuid\n"
  "uint8[] data\n"
  "string body";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__Res__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__Res__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 36, 36},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__Res__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__Res__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
