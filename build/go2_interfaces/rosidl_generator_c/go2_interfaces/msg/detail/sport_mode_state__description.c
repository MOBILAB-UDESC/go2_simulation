// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/SportModeState.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/sport_mode_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__SportModeState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xed, 0x9a, 0xf7, 0xea, 0x5a, 0x53, 0xc9, 0x32,
      0xb1, 0x2b, 0xcc, 0xce, 0xd6, 0xe6, 0x6c, 0x69,
      0x7a, 0xa3, 0x03, 0xf5, 0x32, 0xe6, 0x31, 0xd5,
      0x3c, 0x75, 0xf3, 0x30, 0x18, 0xee, 0x54, 0x18,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "go2_interfaces/msg/detail/time_spec__functions.h"
#include "go2_interfaces/msg/detail/imu_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t go2_interfaces__msg__IMUState__EXPECTED_HASH = {1, {
    0x92, 0x9f, 0xf1, 0x19, 0x89, 0x06, 0x48, 0x87,
    0x14, 0xe3, 0x68, 0xaa, 0x12, 0x25, 0xe0, 0x13,
    0x5e, 0xb3, 0xed, 0xaf, 0x94, 0x0c, 0x91, 0x0e,
    0xe2, 0x0c, 0x33, 0xc2, 0x12, 0x5d, 0x1e, 0xad,
  }};
static const rosidl_type_hash_t go2_interfaces__msg__TimeSpec__EXPECTED_HASH = {1, {
    0xa8, 0x8c, 0xea, 0x87, 0xa0, 0x24, 0x0f, 0x05,
    0xe2, 0x4a, 0xf1, 0x1f, 0xd7, 0x9b, 0x2d, 0xf9,
    0x74, 0x73, 0xc6, 0x30, 0x88, 0x1a, 0x44, 0x9d,
    0x3f, 0xbc, 0x8b, 0x8c, 0x6d, 0x63, 0xbb, 0x86,
  }};
#endif

static char go2_interfaces__msg__SportModeState__TYPE_NAME[] = "go2_interfaces/msg/SportModeState";
static char go2_interfaces__msg__IMUState__TYPE_NAME[] = "go2_interfaces/msg/IMUState";
static char go2_interfaces__msg__TimeSpec__TYPE_NAME[] = "go2_interfaces/msg/TimeSpec";

// Define type names, field names, and default values
static char go2_interfaces__msg__SportModeState__FIELD_NAME__stamp[] = "stamp";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__error_code[] = "error_code";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__imu_state[] = "imu_state";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__mode[] = "mode";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__progress[] = "progress";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__gait_type[] = "gait_type";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__foot_raise_height[] = "foot_raise_height";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__position[] = "position";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__body_height[] = "body_height";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__velocity[] = "velocity";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__yaw_speed[] = "yaw_speed";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__range_obstacle[] = "range_obstacle";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__foot_force[] = "foot_force";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__foot_position_body[] = "foot_position_body";
static char go2_interfaces__msg__SportModeState__FIELD_NAME__foot_speed_body[] = "foot_speed_body";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__SportModeState__FIELDS[] = {
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {go2_interfaces__msg__TimeSpec__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__error_code, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__imu_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {go2_interfaces__msg__IMUState__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__progress, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__gait_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__foot_raise_height, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__body_height, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__yaw_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__range_obstacle, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__foot_force, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__foot_position_body, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__SportModeState__FIELD_NAME__foot_speed_body, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription go2_interfaces__msg__SportModeState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {go2_interfaces__msg__IMUState__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__TimeSpec__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__SportModeState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__SportModeState__TYPE_NAME, 33, 33},
      {go2_interfaces__msg__SportModeState__FIELDS, 15, 15},
    },
    {go2_interfaces__msg__SportModeState__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&go2_interfaces__msg__IMUState__EXPECTED_HASH, go2_interfaces__msg__IMUState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = go2_interfaces__msg__IMUState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&go2_interfaces__msg__TimeSpec__EXPECTED_HASH, go2_interfaces__msg__TimeSpec__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = go2_interfaces__msg__TimeSpec__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "TimeSpec stamp\n"
  "uint32 error_code\n"
  "IMUState imu_state\n"
  "uint8 mode\n"
  "float32 progress\n"
  "uint8 gait_type\n"
  "float32 foot_raise_height\n"
  "float32[3] position\n"
  "float32 body_height\n"
  "float32[3] velocity\n"
  "float32 yaw_speed\n"
  "float32[4] range_obstacle\n"
  "int16[4] foot_force\n"
  "float32[12] foot_position_body\n"
  "float32[12] foot_speed_body";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__SportModeState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__SportModeState__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 304, 304},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__SportModeState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__SportModeState__get_individual_type_description_source(NULL),
    sources[1] = *go2_interfaces__msg__IMUState__get_individual_type_description_source(NULL);
    sources[2] = *go2_interfaces__msg__TimeSpec__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
