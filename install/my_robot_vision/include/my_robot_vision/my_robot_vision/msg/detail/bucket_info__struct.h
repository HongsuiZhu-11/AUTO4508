// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/msg/bucket_info.h"


#ifndef MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__STRUCT_H_
#define MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'color'
// Member 'image_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BucketInfo in the package my_robot_vision.
typedef struct my_robot_vision__msg__BucketInfo
{
  rosidl_runtime_c__String color;
  rosidl_runtime_c__String image_path;
  double distance_to_cone;
  double x;
  double y;
} my_robot_vision__msg__BucketInfo;

// Struct for a sequence of my_robot_vision__msg__BucketInfo.
typedef struct my_robot_vision__msg__BucketInfo__Sequence
{
  my_robot_vision__msg__BucketInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_vision__msg__BucketInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__STRUCT_H_
