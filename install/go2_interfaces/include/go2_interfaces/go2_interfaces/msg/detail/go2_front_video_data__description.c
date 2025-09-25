// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/go2_front_video_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__Go2FrontVideoData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe8, 0x42, 0x60, 0xce, 0xc2, 0x50, 0x66, 0x96,
      0x7b, 0x1b, 0x01, 0x1b, 0x31, 0xc2, 0x81, 0x80,
      0x1f, 0xba, 0x4c, 0x1f, 0xcb, 0x7c, 0x37, 0x07,
      0xd5, 0x9d, 0x47, 0xc8, 0x96, 0x1b, 0x04, 0x2a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__Go2FrontVideoData__TYPE_NAME[] = "go2_interfaces/msg/Go2FrontVideoData";

// Define type names, field names, and default values
static char go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__time_frame[] = "time_frame";
static char go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__video720p[] = "video720p";
static char go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__video360p[] = "video360p";
static char go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__video180p[] = "video180p";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__Go2FrontVideoData__FIELDS[] = {
  {
    {go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__time_frame, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__video720p, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__video360p, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__Go2FrontVideoData__FIELD_NAME__video180p, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__Go2FrontVideoData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__Go2FrontVideoData__TYPE_NAME, 36, 36},
      {go2_interfaces__msg__Go2FrontVideoData__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 time_frame\n"
  "uint8[] video720p\n"
  "uint8[] video360p\n"
  "uint8[] video180p";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__Go2FrontVideoData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__Go2FrontVideoData__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 71, 71},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__Go2FrontVideoData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__Go2FrontVideoData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
