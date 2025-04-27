// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Gpsx.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/gpsx.h"


#ifndef INTERFACES__MSG__DETAIL__GPSX__STRUCT_H_
#define INTERFACES__MSG__DETAIL__GPSX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Gpsx in the package interfaces.
typedef struct interfaces__msg__Gpsx
{
  double longitude;
  double latitude;
  double altitude;
  float ground_speed;
  uint8_t satellites;
  uint8_t mode_indicator;
  float separation;
  float true_course;
  float true_course_magnetic;
  float dilution;
  uint32_t utc_time;
} interfaces__msg__Gpsx;

// Struct for a sequence of interfaces__msg__Gpsx.
typedef struct interfaces__msg__Gpsx__Sequence
{
  interfaces__msg__Gpsx * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Gpsx__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__GPSX__STRUCT_H_
