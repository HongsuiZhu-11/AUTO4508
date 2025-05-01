// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice
#include "my_robot_vision/msg/detail/bucket_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `color`
// Member `image_path`
#include "rosidl_runtime_c/string_functions.h"

bool
my_robot_vision__msg__BucketInfo__init(my_robot_vision__msg__BucketInfo * msg)
{
  if (!msg) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    my_robot_vision__msg__BucketInfo__fini(msg);
    return false;
  }
  // image_path
  if (!rosidl_runtime_c__String__init(&msg->image_path)) {
    my_robot_vision__msg__BucketInfo__fini(msg);
    return false;
  }
  // distance_to_cone
  // x
  // y
  return true;
}

void
my_robot_vision__msg__BucketInfo__fini(my_robot_vision__msg__BucketInfo * msg)
{
  if (!msg) {
    return;
  }
  // color
  rosidl_runtime_c__String__fini(&msg->color);
  // image_path
  rosidl_runtime_c__String__fini(&msg->image_path);
  // distance_to_cone
  // x
  // y
}

bool
my_robot_vision__msg__BucketInfo__are_equal(const my_robot_vision__msg__BucketInfo * lhs, const my_robot_vision__msg__BucketInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  // image_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->image_path), &(rhs->image_path)))
  {
    return false;
  }
  // distance_to_cone
  if (lhs->distance_to_cone != rhs->distance_to_cone) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
my_robot_vision__msg__BucketInfo__copy(
  const my_robot_vision__msg__BucketInfo * input,
  my_robot_vision__msg__BucketInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  // image_path
  if (!rosidl_runtime_c__String__copy(
      &(input->image_path), &(output->image_path)))
  {
    return false;
  }
  // distance_to_cone
  output->distance_to_cone = input->distance_to_cone;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

my_robot_vision__msg__BucketInfo *
my_robot_vision__msg__BucketInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_vision__msg__BucketInfo * msg = (my_robot_vision__msg__BucketInfo *)allocator.allocate(sizeof(my_robot_vision__msg__BucketInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_robot_vision__msg__BucketInfo));
  bool success = my_robot_vision__msg__BucketInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_robot_vision__msg__BucketInfo__destroy(my_robot_vision__msg__BucketInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_robot_vision__msg__BucketInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_robot_vision__msg__BucketInfo__Sequence__init(my_robot_vision__msg__BucketInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_vision__msg__BucketInfo * data = NULL;

  if (size) {
    data = (my_robot_vision__msg__BucketInfo *)allocator.zero_allocate(size, sizeof(my_robot_vision__msg__BucketInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_robot_vision__msg__BucketInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_robot_vision__msg__BucketInfo__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_robot_vision__msg__BucketInfo__Sequence__fini(my_robot_vision__msg__BucketInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_robot_vision__msg__BucketInfo__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_robot_vision__msg__BucketInfo__Sequence *
my_robot_vision__msg__BucketInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_robot_vision__msg__BucketInfo__Sequence * array = (my_robot_vision__msg__BucketInfo__Sequence *)allocator.allocate(sizeof(my_robot_vision__msg__BucketInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_robot_vision__msg__BucketInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_robot_vision__msg__BucketInfo__Sequence__destroy(my_robot_vision__msg__BucketInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_robot_vision__msg__BucketInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_robot_vision__msg__BucketInfo__Sequence__are_equal(const my_robot_vision__msg__BucketInfo__Sequence * lhs, const my_robot_vision__msg__BucketInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_robot_vision__msg__BucketInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_robot_vision__msg__BucketInfo__Sequence__copy(
  const my_robot_vision__msg__BucketInfo__Sequence * input,
  my_robot_vision__msg__BucketInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_robot_vision__msg__BucketInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_robot_vision__msg__BucketInfo * data =
      (my_robot_vision__msg__BucketInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_robot_vision__msg__BucketInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_robot_vision__msg__BucketInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_robot_vision__msg__BucketInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
