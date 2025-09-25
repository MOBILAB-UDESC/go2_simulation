// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/motor_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__MotorState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7a, 0x42, 0xfb, 0xae, 0x9f, 0xad, 0xf7, 0x51,
      0xed, 0xda, 0xb2, 0x43, 0xd1, 0xa9, 0xab, 0xb5,
      0x4c, 0xed, 0xd4, 0x23, 0x9e, 0x43, 0xa1, 0x75,
      0x17, 0xb6, 0xba, 0x76, 0xc1, 0x08, 0x14, 0x3a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__MotorState__TYPE_NAME[] = "go2_interfaces/msg/MotorState";

// Define type names, field names, and default values
static char go2_interfaces__msg__MotorState__FIELD_NAME__mode[] = "mode";
static char go2_interfaces__msg__MotorState__FIELD_NAME__q[] = "q";
static char go2_interfaces__msg__MotorState__FIELD_NAME__dq[] = "dq";
static char go2_interfaces__msg__MotorState__FIELD_NAME__ddq[] = "ddq";
static char go2_interfaces__msg__MotorState__FIELD_NAME__tau_est[] = "tau_est";
static char go2_interfaces__msg__MotorState__FIELD_NAME__q_raw[] = "q_raw";
static char go2_interfaces__msg__MotorState__FIELD_NAME__dq_raw[] = "dq_raw";
static char go2_interfaces__msg__MotorState__FIELD_NAME__ddq_raw[] = "ddq_raw";
static char go2_interfaces__msg__MotorState__FIELD_NAME__temperature[] = "temperature";
static char go2_interfaces__msg__MotorState__FIELD_NAME__lost[] = "lost";
static char go2_interfaces__msg__MotorState__FIELD_NAME__reserve[] = "reserve";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__MotorState__FIELDS[] = {
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__dq, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__ddq, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__tau_est, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__q_raw, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__dq_raw, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__ddq_raw, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__lost, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__MotorState__FIELD_NAME__reserve, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__MotorState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__MotorState__TYPE_NAME, 29, 29},
      {go2_interfaces__msg__MotorState__FIELDS, 11, 11},
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
  "float32 q\n"
  "float32 dq\n"
  "float32 ddq\n"
  "float32 tau_est\n"
  "float32 q_raw\n"
  "float32 dq_raw\n"
  "float32 ddq_raw\n"
  "int8 temperature\n"
  "uint32 lost\n"
  "uint32[2] reserve";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__MotorState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__MotorState__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 151, 151},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__MotorState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__MotorState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
