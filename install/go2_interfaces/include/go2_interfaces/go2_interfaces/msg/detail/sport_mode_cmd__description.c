// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/SportModeCmd.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/sport_mode_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__SportModeCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x87, 0x86, 0xb6, 0x03, 0x54, 0x71, 0xba, 0x2c,
      0x46, 0xa5, 0x36, 0xe0, 0x3e, 0xc9, 0xb7, 0x6f,
      0x59, 0xae, 0x20, 0x22, 0x4b, 0x4e, 0x6d, 0x16,
      0x2c, 0xd0, 0x17, 0x67, 0x91, 0x46, 0x7a, 0x86,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "go2_interfaces/msg/detail/bms_cmd__functions.h"
#include "go2_interfaces/msg/detail/path_point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t go2_interfaces__msg__BmsCmd__EXPECTED_HASH = {1, {
    0x7a, 0x91, 0x48, 0x78, 0xe8, 0xdb, 0x66, 0x2e,
    0x0c, 0x41, 0xd5, 0x54, 0x83, 0x2e, 0x25, 0xa2,
    0xc4, 0x1e, 0xfc, 0xcd, 0xe1, 0x4e, 0xe0, 0xd2,
    0x23, 0x38, 0xca, 0xcb, 0x13, 0x99, 0xeb, 0x08,
  }};
static const rosidl_type_hash_t go2_interfaces__msg__PathPoint__EXPECTED_HASH = {1, {
    0x61, 0x54, 0xa9, 0xc8, 0x32, 0xb3, 0xdd, 0xac,
    0xb9, 0x55, 0xa2, 0xaf, 0xe0, 0xda, 0x27, 0x3f,
    0x78, 0xfa, 0x85, 0xe8, 0x07, 0xdc, 0xe6, 0x98,
    0xe9, 0xb4, 0x14, 0xa1, 0xcd, 0x78, 0xe8, 0x87,
  }};
#endif

static char go2_interfaces__msg__SportModeCmd__TYPE_NAME[] = "go2_interfaces/msg/SportModeCmd";
static char go2_interfaces__msg__BmsCmd__TYPE_NAME[] = "go2_interfaces/msg/BmsCmd";
static char go2_interfaces__msg__PathPoint__TYPE_NAME[] = "go2_interfaces/msg/PathPoint";

// Define type names, field names, and default values
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__mode[] = "mode";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__gait_type[] = "gait_type";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__speed_level[] = "speed_level";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__foot_raise_height[] = "foot_raise_height";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__body_height[] = "body_height";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__position[] = "position";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__euler[] = "euler";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__velocity[] = "velocity";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__yaw_speed[] = "yaw_speed";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__bms_cmd[] = "bms_cmd";
static char go2_interfaces__msg__SportModeCmd__FIELD_NAME__path_point[] = "path_point";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__SportModeCmd__FIELDS[] = {
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__gait_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__speed_level, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__foot_raise_height, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__body_height, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__euler, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__yaw_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__bms_cmd, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {go2_interfaces__msg__BmsCmd__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeCmd__FIELD_NAME__path_point, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      30,
      0,
      {go2_interfaces__msg__PathPoint__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription go2_interfaces__msg__SportModeCmd__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {go2_interfaces__msg__BmsCmd__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__SportModeCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__SportModeCmd__TYPE_NAME, 31, 31},
      {go2_interfaces__msg__SportModeCmd__FIELDS, 11, 11},
    },
    {go2_interfaces__msg__SportModeCmd__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&go2_interfaces__msg__BmsCmd__EXPECTED_HASH, go2_interfaces__msg__BmsCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = go2_interfaces__msg__BmsCmd__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&go2_interfaces__msg__PathPoint__EXPECTED_HASH, go2_interfaces__msg__PathPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = go2_interfaces__msg__PathPoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 mode\n"
  "uint8 gait_type\n"
  "uint8 speed_level\n"
  "float32 foot_raise_height\n"
  "float32 body_height\n"
  "float32[2] position\n"
  "float32[3] euler\n"
  "float32[2] velocity\n"
  "float32 yaw_speed\n"
  "BmsCmd bms_cmd\n"
  "PathPoint[30] path_point";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__SportModeCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__SportModeCmd__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 205, 205},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__SportModeCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__SportModeCmd__get_individual_type_description_source(NULL),
    sources[1] = *go2_interfaces__msg__BmsCmd__get_individual_type_description_source(NULL);
    sources[2] = *go2_interfaces__msg__PathPoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
