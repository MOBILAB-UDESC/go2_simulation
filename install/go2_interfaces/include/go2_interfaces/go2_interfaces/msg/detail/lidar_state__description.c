// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from go2_interfaces:msg/LidarState.idl
// generated code does not contain a copyright notice

#include "go2_interfaces/msg/detail/lidar_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_go2_interfaces
const rosidl_type_hash_t *
go2_interfaces__msg__LidarState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x26, 0xec, 0xda, 0x8d, 0x38, 0x80, 0x91, 0x4c,
      0x62, 0x6e, 0x4d, 0x07, 0xf0, 0xd7, 0xf8, 0x0d,
      0xe0, 0xa8, 0x12, 0xa3, 0x88, 0x12, 0x67, 0x95,
      0xee, 0xee, 0x9d, 0x23, 0x8b, 0x05, 0x1a, 0x03,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char go2_interfaces__msg__LidarState__TYPE_NAME[] = "go2_interfaces/msg/LidarState";

// Define type names, field names, and default values
static char go2_interfaces__msg__LidarState__FIELD_NAME__stamp[] = "stamp";
static char go2_interfaces__msg__LidarState__FIELD_NAME__firmware_version[] = "firmware_version";
static char go2_interfaces__msg__LidarState__FIELD_NAME__software_version[] = "software_version";
static char go2_interfaces__msg__LidarState__FIELD_NAME__sdk_version[] = "sdk_version";
static char go2_interfaces__msg__LidarState__FIELD_NAME__sys_rotation_speed[] = "sys_rotation_speed";
static char go2_interfaces__msg__LidarState__FIELD_NAME__com_rotation_speed[] = "com_rotation_speed";
static char go2_interfaces__msg__LidarState__FIELD_NAME__error_state[] = "error_state";
static char go2_interfaces__msg__LidarState__FIELD_NAME__cloud_frequency[] = "cloud_frequency";
static char go2_interfaces__msg__LidarState__FIELD_NAME__cloud_packet_loss_rate[] = "cloud_packet_loss_rate";
static char go2_interfaces__msg__LidarState__FIELD_NAME__cloud_size[] = "cloud_size";
static char go2_interfaces__msg__LidarState__FIELD_NAME__cloud_scan_num[] = "cloud_scan_num";
static char go2_interfaces__msg__LidarState__FIELD_NAME__imu_frequency[] = "imu_frequency";
static char go2_interfaces__msg__LidarState__FIELD_NAME__imu_packet_loss_rate[] = "imu_packet_loss_rate";
static char go2_interfaces__msg__LidarState__FIELD_NAME__imu_rpy[] = "imu_rpy";
static char go2_interfaces__msg__LidarState__FIELD_NAME__serial_recv_stamp[] = "serial_recv_stamp";
static char go2_interfaces__msg__LidarState__FIELD_NAME__serial_buffer_size[] = "serial_buffer_size";
static char go2_interfaces__msg__LidarState__FIELD_NAME__serial_buffer_read[] = "serial_buffer_read";

static rosidl_runtime_c__type_description__Field go2_interfaces__msg__LidarState__FIELDS[] = {
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__firmware_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__software_version, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__sdk_version, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__sys_rotation_speed, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__com_rotation_speed, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__error_state, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__cloud_frequency, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__cloud_packet_loss_rate, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__cloud_size, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__cloud_scan_num, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__imu_frequency, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__imu_packet_loss_rate, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__imu_rpy, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__serial_recv_stamp, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__serial_buffer_size, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {go2_interfaces__msg__LidarState__FIELD_NAME__serial_buffer_read, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
go2_interfaces__msg__LidarState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {go2_interfaces__msg__LidarState__TYPE_NAME, 29, 29},
      {go2_interfaces__msg__LidarState__FIELDS, 17, 17},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 stamp\n"
  "string firmware_version\n"
  "string software_version\n"
  "string sdk_version\n"
  "float32 sys_rotation_speed\n"
  "float32 com_rotation_speed\n"
  "uint8 error_state\n"
  "float32 cloud_frequency\n"
  "float32 cloud_packet_loss_rate\n"
  "uint32 cloud_size\n"
  "uint32 cloud_scan_num\n"
  "float32 imu_frequency\n"
  "float32 imu_packet_loss_rate\n"
  "float32[3] imu_rpy\n"
  "float64 serial_recv_stamp\n"
  "uint32 serial_buffer_size\n"
  "uint32 serial_buffer_read";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
go2_interfaces__msg__LidarState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {go2_interfaces__msg__LidarState__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 395, 395},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
go2_interfaces__msg__LidarState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *go2_interfaces__msg__LidarState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
