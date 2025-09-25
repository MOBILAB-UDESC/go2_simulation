// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/IMUState.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/imu_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__IMUState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x92, 0x9f, 0xf1, 0x19, 0x89, 0x06, 0x48, 0x87,
      0x14, 0xe3, 0x68, 0xaa, 0x12, 0x25, 0xe0, 0x13,
      0x5e, 0xb3, 0xed, 0xaf, 0x94, 0x0c, 0x91, 0x0e,
      0xe2, 0x0c, 0x33, 0xc2, 0x12, 0x5d, 0x1e, 0xad,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__IMUState__TYPE_NAME[] = "go2_interfaces/msg/IMUState";

// Define type names, field names, and default values
static char go2_interfaces__msg__IMUState__FIELD_NAME__quaternion[] = "quaternion";
static char go2_interfaces__msg__IMUState__FIELD_NAME__gyroscope[] = "gyroscope";
static char go2_interfaces__msg__IMUState__FIELD_NAME__accelerometer[] = "accelerometer";
static char go2_interfaces__msg__IMUState__FIELD_NAME__rpy[] = "rpy";
static char go2_interfaces__msg__IMUState__FIELD_NAME__temperature[] = "temperature";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__IMUState__FIELDS[] = {
  {
    {go2_interfaces__msg__IMUState__FIELD_NAME__quaternion, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__IMUState__FIELD_NAME__gyroscope, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__IMUState__FIELD_NAME__accelerometer, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__IMUState__FIELD_NAME__rpy, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__IMUState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__IMUState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__IMUState__TYPE_NAME, 27, 27},
      {go2_interfaces__msg__IMUState__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32[4] quaternion\n"
  "float32[3] gyroscope\n"
  "float32[3] accelerometer\n"
  "float32[3] rpy\n"
  "int8 temperature";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__IMUState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__IMUState__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 99, 99},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__IMUState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__IMUState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
