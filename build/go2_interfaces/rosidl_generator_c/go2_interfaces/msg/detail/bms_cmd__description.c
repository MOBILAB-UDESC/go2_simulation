// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/BmsCmd.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/bms_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__BmsCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7a, 0x91, 0x48, 0x78, 0xe8, 0xdb, 0x66, 0x2e,
      0x0c, 0x41, 0xd5, 0x54, 0x83, 0x2e, 0x25, 0xa2,
      0xc4, 0x1e, 0xfc, 0xcd, 0xe1, 0x4e, 0xe0, 0xd2,
      0x23, 0x38, 0xca, 0xcb, 0x13, 0x99, 0xeb, 0x08,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__BmsCmd__TYPE_NAME[] = "go2_interfaces/msg/BmsCmd";

// Define type names, field names, and default values
static char go2_interfaces__msg__BmsCmd__FIELD_NAME__off[] = "off";
static char go2_interfaces__msg__BmsCmd__FIELD_NAME__reserve[] = "reserve";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__BmsCmd__FIELDS[] = {
  {
    {go2_interfaces__msg__BmsCmd__FIELD_NAME__off, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsCmd__FIELD_NAME__reserve, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__BmsCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__BmsCmd__TYPE_NAME, 25, 25},
      {go2_interfaces__msg__BmsCmd__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 off\n"
  "uint8[3] reserve";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__BmsCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__BmsCmd__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 26, 26},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__BmsCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__BmsCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
