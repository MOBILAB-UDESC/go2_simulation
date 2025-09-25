// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/HeightMap.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/height_map__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__HeightMap__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0x6c, 0x68, 0x96, 0xf6, 0x9f, 0x63, 0xa8,
      0x22, 0x67, 0x5c, 0xb2, 0x8e, 0x8b, 0x5a, 0xc0,
      0x86, 0x34, 0xfb, 0xaa, 0xc7, 0xde, 0xf9, 0xf9,
      0xa3, 0x33, 0x04, 0x02, 0x22, 0xe6, 0x09, 0x4f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__HeightMap__TYPE_NAME[] = "go2_interfaces/msg/HeightMap";

// Define type names, field names, and default values
static char go2_interfaces__msg__HeightMap__FIELD_NAME__stamp[] = "stamp";
static char go2_interfaces__msg__HeightMap__FIELD_NAME__frame_id[] = "frame_id";
static char go2_interfaces__msg__HeightMap__FIELD_NAME__resolution[] = "resolution";
static char go2_interfaces__msg__HeightMap__FIELD_NAME__width[] = "width";
static char go2_interfaces__msg__HeightMap__FIELD_NAME__height[] = "height";
static char go2_interfaces__msg__HeightMap__FIELD_NAME__origin[] = "origin";
static char go2_interfaces__msg__HeightMap__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__HeightMap__FIELDS[] = {
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__frame_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__resolution, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__width, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__height, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__origin, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__HeightMap__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__HeightMap__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__HeightMap__TYPE_NAME, 28, 28},
      {go2_interfaces__msg__HeightMap__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Header\n"
  "float64 stamp         # timestamp\n"
  "string frame_id      # world frame id\n"
  "\n"
  "# Map info\n"
  "float32 resolution     # The map resolution [m/cell]\n"
  "uint32 width  # Map width along x-axis [cells]\n"
  "uint32 height # Map height alonge y-axis [cells]\n"
  "float32[2] origin      # Map frame origin xy-position [m], the xyz-axis direction of map frame is aligned with the world frame\n"
  "\n"
  "# Map data, in x-major order, starting with [0,0], ending with [width, height]\n"
  "# For a cell whose 2d-array-index is [ix, iy]\\xef\\xbc\\x8c\n"
  "#    its position in world frame is: [ix * resolution + origin[0], iy * resolution + origin[1]]\n"
  "#    its cell value is: data[width * iy + ix]\n"
  "float32[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__HeightMap__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__HeightMap__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 652, 652},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__HeightMap__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__HeightMap__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
