// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/Req.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/req__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__Req__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0x96, 0xe1, 0x73, 0x1e, 0xad, 0xea, 0x74,
      0x2b, 0xf2, 0xc0, 0xb8, 0xc5, 0xaf, 0xb3, 0x31,
      0xb8, 0x6a, 0xe3, 0x26, 0x4e, 0x21, 0x6a, 0x53,
      0x08, 0xb0, 0x52, 0xf7, 0xae, 0x72, 0xe5, 0xf2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__Req__TYPE_NAME[] = "go2_interfaces/msg/Req";

// Define type names, field names, and default values
static char go2_interfaces__msg__Req__FIELD_NAME__uuid[] = "uuid";
static char go2_interfaces__msg__Req__FIELD_NAME__body[] = "body";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__Req__FIELDS[] = {
  {
    {go2_interfaces__msg__Req__FIELD_NAME__uuid, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Req__FIELD_NAME__body, 4, 4},
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
go2_interfaces__msg__Req__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__Req__TYPE_NAME, 22, 22},
      {go2_interfaces__msg__Req__FIELDS, 2, 2},
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
  "string body";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__Req__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__Req__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 23, 23},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__Req__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__Req__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
