// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/InterfaceConfig.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/interface_config__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__InterfaceConfig__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x25, 0xea, 0x46, 0x1d, 0xc6, 0x02, 0x30, 0xc7,
      0xbd, 0x01, 0x20, 0xcd, 0x65, 0x1d, 0xfe, 0x90,
      0xf3, 0x3e, 0x86, 0x7e, 0xbd, 0x2b, 0x72, 0xa7,
      0xe5, 0xef, 0x3f, 0x06, 0xb3, 0x1c, 0xc7, 0xf4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__InterfaceConfig__TYPE_NAME[] = "go2_interfaces/msg/InterfaceConfig";

// Define type names, field names, and default values
static char go2_interfaces__msg__InterfaceConfig__FIELD_NAME__mode[] = "mode";
static char go2_interfaces__msg__InterfaceConfig__FIELD_NAME__value[] = "value";
static char go2_interfaces__msg__InterfaceConfig__FIELD_NAME__reserve[] = "reserve";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__InterfaceConfig__FIELDS[] = {
  {
    {go2_interfaces__msg__InterfaceConfig__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__InterfaceConfig__FIELD_NAME__value, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__InterfaceConfig__FIELD_NAME__reserve, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__InterfaceConfig__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__InterfaceConfig__TYPE_NAME, 34, 34},
      {go2_interfaces__msg__InterfaceConfig__FIELDS, 3, 3},
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
  "uint8 value\n"
  "uint8[2] reserve";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__InterfaceConfig__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__InterfaceConfig__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 39, 39},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__InterfaceConfig__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__InterfaceConfig__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
