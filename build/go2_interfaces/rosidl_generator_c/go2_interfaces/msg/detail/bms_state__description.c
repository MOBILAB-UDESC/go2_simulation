// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/BmsState.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/bms_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__BmsState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x57, 0x7b, 0xad, 0xb1, 0x75, 0x43, 0xb1, 0x64,
      0x22, 0x68, 0xe7, 0xe5, 0x40, 0x17, 0x8d, 0xdb,
      0x05, 0x45, 0x36, 0x6f, 0xe2, 0xe8, 0x9b, 0xb4,
      0x33, 0x68, 0x3c, 0xb9, 0x9b, 0xef, 0x12, 0xf5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__BmsState__TYPE_NAME[] = "go2_interfaces/msg/BmsState";

// Define type names, field names, and default values
static char go2_interfaces__msg__BmsState__FIELD_NAME__version_high[] = "version_high";
static char go2_interfaces__msg__BmsState__FIELD_NAME__version_low[] = "version_low";
static char go2_interfaces__msg__BmsState__FIELD_NAME__status[] = "status";
static char go2_interfaces__msg__BmsState__FIELD_NAME__soc[] = "soc";
static char go2_interfaces__msg__BmsState__FIELD_NAME__current[] = "current";
static char go2_interfaces__msg__BmsState__FIELD_NAME__cycle[] = "cycle";
static char go2_interfaces__msg__BmsState__FIELD_NAME__bq_ntc[] = "bq_ntc";
static char go2_interfaces__msg__BmsState__FIELD_NAME__mcu_ntc[] = "mcu_ntc";
static char go2_interfaces__msg__BmsState__FIELD_NAME__cell_vol[] = "cell_vol";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__BmsState__FIELDS[] = {
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__version_high, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__version_low, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__soc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__current, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__cycle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__bq_ntc, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__mcu_ntc, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__BmsState__FIELD_NAME__cell_vol, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      15,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__BmsState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__BmsState__TYPE_NAME, 27, 27},
      {go2_interfaces__msg__BmsState__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 version_high\n"
  "uint8 version_low\n"
  "uint8 status\n"
  "uint8 soc\n"
  "int32 current\n"
  "uint16 cycle\n"
  "int8[2] bq_ntc\n"
  "int8[2] mcu_ntc\n"
  "uint16[15] cell_vol";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__BmsState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__BmsState__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 137, 137},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__BmsState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__BmsState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
