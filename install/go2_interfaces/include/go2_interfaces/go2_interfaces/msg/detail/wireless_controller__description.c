// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/WirelessController.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/wireless_controller__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__WirelessController__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb2, 0x01, 0x4e, 0xee, 0x02, 0x1c, 0x7d, 0x4a,
      0xdc, 0x7a, 0x88, 0xe6, 0x56, 0xfc, 0xd3, 0xa4,
      0x85, 0x26, 0x72, 0x47, 0xdd, 0xe0, 0xd7, 0xbf,
      0x84, 0x91, 0x27, 0xef, 0xc1, 0xdc, 0x7b, 0x92,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__WirelessController__TYPE_NAME[] = "go2_interfaces/msg/WirelessController";

// Define type names, field names, and default values
static char go2_interfaces__msg__WirelessController__FIELD_NAME__lx[] = "lx";
static char go2_interfaces__msg__WirelessController__FIELD_NAME__ly[] = "ly";
static char go2_interfaces__msg__WirelessController__FIELD_NAME__rx[] = "rx";
static char go2_interfaces__msg__WirelessController__FIELD_NAME__ry[] = "ry";
static char go2_interfaces__msg__WirelessController__FIELD_NAME__keys[] = "keys";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__WirelessController__FIELDS[] = {
  {
    {go2_interfaces__msg__WirelessController__FIELD_NAME__lx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__WirelessController__FIELD_NAME__ly, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__WirelessController__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__WirelessController__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__WirelessController__FIELD_NAME__keys, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__WirelessController__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__WirelessController__TYPE_NAME, 37, 37},
      {go2_interfaces__msg__WirelessController__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 lx\n"
  "float32 ly\n"
  "float32 rx\n"
  "float32 ry\n"
  "uint16 keys";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__WirelessController__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__WirelessController__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 55, 55},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__WirelessController__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__WirelessController__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
