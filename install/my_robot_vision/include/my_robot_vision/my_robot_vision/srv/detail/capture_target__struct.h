// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_vision:srv/CaptureTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/srv/capture_target.h"


#ifndef MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__STRUCT_H_
#define MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CaptureTarget in the package my_robot_vision.
typedef struct my_robot_vision__srv__CaptureTarget_Request
{
  double robot_x;
  double robot_y;
  double robot_theta_deg;
} my_robot_vision__srv__CaptureTarget_Request;

// Struct for a sequence of my_robot_vision__srv__CaptureTarget_Request.
typedef struct my_robot_vision__srv__CaptureTarget_Request__Sequence
{
  my_robot_vision__srv__CaptureTarget_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_vision__srv__CaptureTarget_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'cone_image_path'
#include "rosidl_runtime_c/string.h"
// Member 'buckets'
#include "my_robot_vision/msg/detail/bucket_info__struct.h"

/// Struct defined in srv/CaptureTarget in the package my_robot_vision.
typedef struct my_robot_vision__srv__CaptureTarget_Response
{
  bool success;
  rosidl_runtime_c__String cone_image_path;
  double world_x;
  double world_y;
  my_robot_vision__msg__BucketInfo__Sequence buckets;
} my_robot_vision__srv__CaptureTarget_Response;

// Struct for a sequence of my_robot_vision__srv__CaptureTarget_Response.
typedef struct my_robot_vision__srv__CaptureTarget_Response__Sequence
{
  my_robot_vision__srv__CaptureTarget_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_vision__srv__CaptureTarget_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  my_robot_vision__srv__CaptureTarget_Event__request__MAX_SIZE = 1
};
// response
enum
{
  my_robot_vision__srv__CaptureTarget_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CaptureTarget in the package my_robot_vision.
typedef struct my_robot_vision__srv__CaptureTarget_Event
{
  service_msgs__msg__ServiceEventInfo info;
  my_robot_vision__srv__CaptureTarget_Request__Sequence request;
  my_robot_vision__srv__CaptureTarget_Response__Sequence response;
} my_robot_vision__srv__CaptureTarget_Event;

// Struct for a sequence of my_robot_vision__srv__CaptureTarget_Event.
typedef struct my_robot_vision__srv__CaptureTarget_Event__Sequence
{
  my_robot_vision__srv__CaptureTarget_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_vision__srv__CaptureTarget_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__STRUCT_H_
