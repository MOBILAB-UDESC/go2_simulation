// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from go2_interfaces:msg/Go2FrontVideoData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "go2_interfaces/msg/go2_front_video_data.h"


#ifndef GO2_INTERFACES__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_H_
#define GO2_INTERFACES__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'video720p'
// Member 'video360p'
// Member 'video180p'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Go2FrontVideoData in the package go2_interfaces.
typedef struct go2_interfaces__msg__Go2FrontVideoData
{
  uint64_t time_frame;
  rosidl_runtime_c__uint8__Sequence video720p;
  rosidl_runtime_c__uint8__Sequence video360p;
  rosidl_runtime_c__uint8__Sequence video180p;
} go2_interfaces__msg__Go2FrontVideoData;

// Struct for a sequence of go2_interfaces__msg__Go2FrontVideoData.
typedef struct go2_interfaces__msg__Go2FrontVideoData__Sequence
{
  go2_interfaces__msg__Go2FrontVideoData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_interfaces__msg__Go2FrontVideoData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GO2_INTERFACES__MSG__DETAIL__GO2_FRONT_VIDEO_DATA__STRUCT_H_
