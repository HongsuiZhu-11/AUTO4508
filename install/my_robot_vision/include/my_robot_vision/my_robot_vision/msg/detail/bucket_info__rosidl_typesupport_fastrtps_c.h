// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice
#ifndef MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "my_robot_vision/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "my_robot_vision/msg/detail/bucket_info__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
bool cdr_serialize_my_robot_vision__msg__BucketInfo(
  const my_robot_vision__msg__BucketInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
bool cdr_deserialize_my_robot_vision__msg__BucketInfo(
  eprosima::fastcdr::Cdr &,
  my_robot_vision__msg__BucketInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
size_t get_serialized_size_my_robot_vision__msg__BucketInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
size_t max_serialized_size_my_robot_vision__msg__BucketInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
bool cdr_serialize_key_my_robot_vision__msg__BucketInfo(
  const my_robot_vision__msg__BucketInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
size_t get_serialized_size_key_my_robot_vision__msg__BucketInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
size_t max_serialized_size_key_my_robot_vision__msg__BucketInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_my_robot_vision
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, my_robot_vision, msg, BucketInfo)();

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
