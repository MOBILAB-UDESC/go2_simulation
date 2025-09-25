// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/UwbSwitch.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/uwb_switch__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__UwbSwitch__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x81, 0x72, 0x0d, 0x97, 0xc7, 0xa2, 0x77, 0xda,
      0x37, 0xb8, 0x5a, 0x3e, 0x8a, 0xc9, 0x35, 0xbf,
      0xe9, 0xd4, 0x7a, 0x1d, 0x7d, 0x3e, 0x3a, 0xd6,
      0xd2, 0xf8, 0x65, 0xe2, 0xcc, 0x76, 0x54, 0x5e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__UwbSwitch__TYPE_NAME[] = "go2_interfaces/msg/UwbSwitch";

// Define type names, field names, and default values
static char go2_interfaces__msg__UwbSwitch__FIELD_NAME__enabled[] = "enabled";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__UwbSwitch__FIELDS[] = {
  {
    {go2_interfaces__msg__UwbSwitch__FIELD_NAME__enabled, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__UwbSwitch__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__UwbSwitch__TYPE_NAME, 28, 28},
      {go2_interfaces__msg__UwbSwitch__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 enabled";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__UwbSwitch__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__UwbSwitch__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 13, 13},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__UwbSwitch__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__UwbSwitch__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
