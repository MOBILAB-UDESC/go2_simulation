// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/LowCmd.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/low_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__LowCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3e, 0x30, 0xbe, 0x3e, 0x51, 0xe1, 0x9f, 0xf4,
      0xcb, 0x6a, 0x5c, 0x65, 0x96, 0x76, 0x6f, 0x8f,
      0xdc, 0x99, 0xf9, 0x5e, 0x25, 0x2a, 0x03, 0x24,
      0xd2, 0xcd, 0x5c, 0xe9, 0xb6, 0xf5, 0x43, 0x7a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "go2_interfaces/msg/detail/motor_cmd__functions.h"
#include "go2_interfaces/msg/detail/bms_cmd__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t go2_interfaces__msg__BmsCmd__EXPECTED_HASH = {1, {
    0x7a, 0x91, 0x48, 0x78, 0xe8, 0xdb, 0x66, 0x2e,
    0x0c, 0x41, 0xd5, 0x54, 0x83, 0x2e, 0x25, 0xa2,
    0xc4, 0x1e, 0xfc, 0xcd, 0xe1, 0x4e, 0xe0, 0xd2,
    0x23, 0x38, 0xca, 0xcb, 0x13, 0x99, 0xeb, 0x08,
  }};
static const rosidl_type_hash_t go2_interfaces__msg__MotorCmd__EXPECTED_HASH = {1, {
    0xa8, 0xa7, 0x7b, 0x38, 0xdc, 0x13, 0xfc, 0x60,
    0xcb, 0xa8, 0x46, 0xe8, 0x98, 0x12, 0xd8, 0xd7,
    0xe6, 0x75, 0x14, 0xd6, 0x68, 0x31, 0x6a, 0x54,
    0xc1, 0xd6, 0xa8, 0x04, 0x9d, 0x29, 0xde, 0x6e,
  }};
#endif

static char go2_interfaces__msg__LowCmd__TYPE_NAME[] = "go2_interfaces/msg/LowCmd";
static char go2_interfaces__msg__BmsCmd__TYPE_NAME[] = "go2_interfaces/msg/BmsCmd";
static char go2_interfaces__msg__MotorCmd__TYPE_NAME[] = "go2_interfaces/msg/MotorCmd";

// Define type names, field names, and default values
static char go2_interfaces__msg__LowCmd__FIELD_NAME__head[] = "head";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__level_flag[] = "level_flag";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__frame_reserve[] = "frame_reserve";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__sn[] = "sn";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__version[] = "version";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__bandwidth[] = "bandwidth";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__motor_cmd[] = "motor_cmd";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__bms_cmd[] = "bms_cmd";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__wireless_remote[] = "wireless_remote";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__led[] = "led";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__fan[] = "fan";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__gpio[] = "gpio";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__reserve[] = "reserve";
static char go2_interfaces__msg__LowCmd__FIELD_NAME__crc[] = "crc";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__LowCmd__FIELDS[] = {
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__head, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__level_flag, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__frame_reserve, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__sn, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__version, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__bandwidth, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__motor_cmd, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      20,
      0,
      {go2_interfaces__msg__MotorCmd__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__bms_cmd, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {go2_interfaces__msg__BmsCmd__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__wireless_remote, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__led, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      12,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__fan, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__gpio, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__reserve, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowCmd__FIELD_NAME__crc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription go2_interfaces__msg__LowCmd__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {go2_interfaces__msg__BmsCmd__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorCmd__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__LowCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__LowCmd__TYPE_NAME, 25, 25},
      {go2_interfaces__msg__LowCmd__FIELDS, 14, 14},
    },
    {go2_interfaces__msg__LowCmd__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&go2_interfaces__msg__BmsCmd__EXPECTED_HASH, go2_interfaces__msg__BmsCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = go2_interfaces__msg__BmsCmd__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&go2_interfaces__msg__MotorCmd__EXPECTED_HASH, go2_interfaces__msg__MotorCmd__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = go2_interfaces__msg__MotorCmd__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8[2] head\n"
  "uint8 level_flag\n"
  "uint8 frame_reserve\n"
  "uint32[2] sn\n"
  "uint32[2] version\n"
  "uint16 bandwidth\n"
  "MotorCmd[20] motor_cmd\n"
  "BmsCmd bms_cmd\n"
  "uint8[40] wireless_remote\n"
  "uint8[12] led\n"
  "uint8[2] fan\n"
  "uint8 gpio\n"
  "uint32 reserve\n"
  "uint32 crc";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__LowCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__LowCmd__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 226, 226},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__LowCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__LowCmd__get_individual_type_description_source(NULL),
    sources[1] = *go2_interfaces__msg__BmsCmd__get_individual_type_description_source(NULL);
    sources[2] = *go2_interfaces__msg__MotorCmd__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
