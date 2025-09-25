// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from go2_interfaces:msg/LowState.idl
// generated code does not contain a copyright notice
#ifndef GO2_INTERFACES__MSG__DETAIL__LOW_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define GO2_INTERFACES__MSG__DETAIL__LOW_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "go2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "go2_interfaces/msg/detail/low_state__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
bool cdr_serialize_go2_interfaces__msg__LowState(
  const go2_interfaces__msg__LowState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
bool cdr_deserialize_go2_interfaces__msg__LowState(
  eprosima::fastcdr::Cdr &,
  go2_interfaces__msg__LowState * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
size_t get_serialized_size_go2_interfaces__msg__LowState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
size_t max_serialized_size_go2_interfaces__msg__LowState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
bool cdr_serialize_key_go2_interfaces__msg__LowState(
  const go2_interfaces__msg__LowState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
size_t get_serialized_size_key_go2_interfaces__msg__LowState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
size_t max_serialized_size_key_go2_interfaces__msg__LowState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_go2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, go2_interfaces, msg, LowState)();

#ifdef __cplusplus
}
#endif

#endif  // GO2_INTERFACES__MSG__DETAIL__LOW_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
