// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/PathPoint.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/path_point__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__PathPoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x61, 0x54, 0xa9, 0xc8, 0x32, 0xb3, 0xdd, 0xac,
      0xb9, 0x55, 0xa2, 0xaf, 0xe0, 0xda, 0x27, 0x3f,
      0x78, 0xfa, 0x85, 0xe8, 0x07, 0xdc, 0xe6, 0x98,
      0xe9, 0xb4, 0x14, 0xa1, 0xcd, 0x78, 0xe8, 0x87,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__PathPoint__TYPE_NAME[] = "go2_interfaces/msg/PathPoint";

// Define type names, field names, and default values
static char go2_interfaces__msg__PathPoint__FIELD_NAME__t_from_start[] = "t_from_start";
static char go2_interfaces__msg__PathPoint__FIELD_NAME__x[] = "x";
static char go2_interfaces__msg__PathPoint__FIELD_NAME__y[] = "y";
static char go2_interfaces__msg__PathPoint__FIELD_NAME__yaw[] = "yaw";
static char go2_interfaces__msg__PathPoint__FIELD_NAME__vx[] = "vx";
static char go2_interfaces__msg__PathPoint__FIELD_NAME__vy[] = "vy";
static char go2_interfaces__msg__PathPoint__FIELD_NAME__vyaw[] = "vyaw";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__PathPoint__FIELDS[] = {
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__t_from_start, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__vx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__vy, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__PathPoint__FIELD_NAME__vyaw, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__PathPoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__PathPoint__TYPE_NAME, 28, 28},
      {go2_interfaces__msg__PathPoint__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 t_from_start\n"
  "float32 x\n"
  "float32 y\n"
  "float32 yaw\n"
  "float32 vx\n"
  "float32 vy\n"
  "float32 vyaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__PathPoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__PathPoint__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 87, 87},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__PathPoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__PathPoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
