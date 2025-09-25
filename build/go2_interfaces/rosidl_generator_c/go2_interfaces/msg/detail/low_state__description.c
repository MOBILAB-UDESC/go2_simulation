// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/LowState.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/low_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__LowState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb7, 0x86, 0x56, 0xf5, 0x87, 0x62, 0xa5, 0x5a,
      0x08, 0xa2, 0x8e, 0x6f, 0xe4, 0x24, 0x01, 0x26,
      0x20, 0x14, 0x69, 0x6f, 0xeb, 0xc9, 0xaf, 0x81,
      0xef, 0x60, 0x9b, 0x75, 0x7d, 0xd9, 0xbf, 0x47,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "go2_interfaces/msg/detail/imu_state__functions.h"
#include "go2_interfaces/msg/detail/motor_state__functions.h"
#include "go2_interfaces/msg/detail/bms_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t go2_interfaces__msg__BmsState__EXPECTED_HASH = {1, {
    0x57, 0x7b, 0xad, 0xb1, 0x75, 0x43, 0xb1, 0x64,
    0x22, 0x68, 0xe7, 0xe5, 0x40, 0x17, 0x8d, 0xdb,
    0x05, 0x45, 0x36, 0x6f, 0xe2, 0xe8, 0x9b, 0xb4,
    0x33, 0x68, 0x3c, 0xb9, 0x9b, 0xef, 0x12, 0xf5,
  }};
static const rosidl_type_hash_t go2_interfaces__msg__IMUState__EXPECTED_HASH = {1, {
    0x92, 0x9f, 0xf1, 0x19, 0x89, 0x06, 0x48, 0x87,
    0x14, 0xe3, 0x68, 0xaa, 0x12, 0x25, 0xe0, 0x13,
    0x5e, 0xb3, 0xed, 0xaf, 0x94, 0x0c, 0x91, 0x0e,
    0xe2, 0x0c, 0x33, 0xc2, 0x12, 0x5d, 0x1e, 0xad,
  }};
static const rosidl_type_hash_t go2_interfaces__msg__MotorState__EXPECTED_HASH = {1, {
    0x7a, 0x42, 0xfb, 0xae, 0x9f, 0xad, 0xf7, 0x51,
    0xed, 0xda, 0xb2, 0x43, 0xd1, 0xa9, 0xab, 0xb5,
    0x4c, 0xed, 0xd4, 0x23, 0x9e, 0x43, 0xa1, 0x75,
    0x17, 0xb6, 0xba, 0x76, 0xc1, 0x08, 0x14, 0x3a,
  }};
#endif

static char go2_interfaces__msg__LowState__TYPE_NAME[] = "go2_interfaces/msg/LowState";
static char go2_interfaces__msg__BmsState__TYPE_NAME[] = "go2_interfaces/msg/BmsState";
static char go2_interfaces__msg__IMUState__TYPE_NAME[] = "go2_interfaces/msg/IMUState";
static char go2_interfaces__msg__MotorState__TYPE_NAME[] = "go2_interfaces/msg/MotorState";

// Define type names, field names, and default values
static char go2_interfaces__msg__LowState__FIELD_NAME__head[] = "head";
static char go2_interfaces__msg__LowState__FIELD_NAME__level_flag[] = "level_flag";
static char go2_interfaces__msg__LowState__FIELD_NAME__frame_reserve[] = "frame_reserve";
static char go2_interfaces__msg__LowState__FIELD_NAME__sn[] = "sn";
static char go2_interfaces__msg__LowState__FIELD_NAME__version[] = "version";
static char go2_interfaces__msg__LowState__FIELD_NAME__bandwidth[] = "bandwidth";
static char go2_interfaces__msg__LowState__FIELD_NAME__imu_state[] = "imu_state";
static char go2_interfaces__msg__LowState__FIELD_NAME__motor_state[] = "motor_state";
static char go2_interfaces__msg__LowState__FIELD_NAME__bms_state[] = "bms_state";
static char go2_interfaces__msg__LowState__FIELD_NAME__foot_force[] = "foot_force";
static char go2_interfaces__msg__LowState__FIELD_NAME__foot_force_est[] = "foot_force_est";
static char go2_interfaces__msg__LowState__FIELD_NAME__tick[] = "tick";
static char go2_interfaces__msg__LowState__FIELD_NAME__wireless_remote[] = "wireless_remote";
static char go2_interfaces__msg__LowState__FIELD_NAME__bit_flag[] = "bit_flag";
static char go2_interfaces__msg__LowState__FIELD_NAME__adc_reel[] = "adc_reel";
static char go2_interfaces__msg__LowState__FIELD_NAME__temperature_ntc1[] = "temperature_ntc1";
static char go2_interfaces__msg__LowState__FIELD_NAME__temperature_ntc2[] = "temperature_ntc2";
static char go2_interfaces__msg__LowState__FIELD_NAME__power_v[] = "power_v";
static char go2_interfaces__msg__LowState__FIELD_NAME__power_a[] = "power_a";
static char go2_interfaces__msg__LowState__FIELD_NAME__fan_frequency[] = "fan_frequency";
static char go2_interfaces__msg__LowState__FIELD_NAME__reserve[] = "reserve";
static char go2_interfaces__msg__LowState__FIELD_NAME__crc[] = "crc";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__LowState__FIELDS[] = {
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__head, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__level_flag, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__frame_reserve, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__sn, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__version, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__bandwidth, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__imu_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {go2_interfaces__msg__IMUState__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__motor_state, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      20,
      0,
      {go2_interfaces__msg__MotorState__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__bms_state, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {go2_interfaces__msg__BmsState__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__foot_force, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__foot_force_est, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__tick, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__wireless_remote, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      40,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__bit_flag, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__adc_reel, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__temperature_ntc1, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__temperature_ntc2, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__power_v, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__power_a, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__fan_frequency, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__reserve, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LowState__FIELD_NAME__crc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription go2_interfaces__msg__LowState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {go2_interfaces__msg__BmsState__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__IMUState__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__LowState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__LowState__TYPE_NAME, 27, 27},
      {go2_interfaces__msg__LowState__FIELDS, 22, 22},
    },
    {go2_interfaces__msg__LowState__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&go2_interfaces__msg__BmsState__EXPECTED_HASH, go2_interfaces__msg__BmsState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = go2_interfaces__msg__BmsState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&go2_interfaces__msg__IMUState__EXPECTED_HASH, go2_interfaces__msg__IMUState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = go2_interfaces__msg__IMUState__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&go2_interfaces__msg__MotorState__EXPECTED_HASH, go2_interfaces__msg__MotorState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = go2_interfaces__msg__MotorState__get_type_description(NULL)->type_description.fields;
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
  "IMUState imu_state\n"
  "MotorState[20] motor_state\n"
  "BmsState bms_state\n"
  "int16[4] foot_force\n"
  "int16[4] foot_force_est\n"
  "uint32 tick\n"
  "uint8[40] wireless_remote\n"
  "uint8 bit_flag\n"
  "float32 adc_reel\n"
  "int8 temperature_ntc1\n"
  "int8 temperature_ntc2\n"
  "float32 power_v\n"
  "float32 power_a\n"
  "uint16[4] fan_frequency\n"
  "uint32 reserve\n"
  "uint32 crc";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__LowState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__LowState__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 403, 403},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__LowState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__LowState__get_individual_type_description_source(NULL),
    sources[1] = *go2_interfaces__msg__BmsState__get_individual_type_description_source(NULL);
    sources[2] = *go2_interfaces__msg__IMUState__get_individual_type_description_source(NULL);
    sources[3] = *go2_interfaces__msg__MotorState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
