// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/msg/bucket_info.h"


#ifndef MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__FUNCTIONS_H_
#define MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "my_robot_vision/msg/rosidl_generator_c__visibility_control.h"

#include "my_robot_vision/msg/detail/bucket_info__struct.h"

/// Initialize msg/BucketInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * my_robot_vision__msg__BucketInfo
 * )) before or use
 * my_robot_vision__msg__BucketInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
bool
my_robot_vision__msg__BucketInfo__init(my_robot_vision__msg__BucketInfo * msg);

/// Finalize msg/BucketInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
void
my_robot_vision__msg__BucketInfo__fini(my_robot_vision__msg__BucketInfo * msg);

/// Create msg/BucketInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * my_robot_vision__msg__BucketInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
my_robot_vision__msg__BucketInfo *
my_robot_vision__msg__BucketInfo__create(void);

/// Destroy msg/BucketInfo message.
/**
 * It calls
 * my_robot_vision__msg__BucketInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
void
my_robot_vision__msg__BucketInfo__destroy(my_robot_vision__msg__BucketInfo * msg);

/// Check for msg/BucketInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
bool
my_robot_vision__msg__BucketInfo__are_equal(const my_robot_vision__msg__BucketInfo * lhs, const my_robot_vision__msg__BucketInfo * rhs);

/// Copy a msg/BucketInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
bool
my_robot_vision__msg__BucketInfo__copy(
  const my_robot_vision__msg__BucketInfo * input,
  my_robot_vision__msg__BucketInfo * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_type_hash_t *
my_robot_vision__msg__BucketInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_runtime_c__type_description__TypeDescription *
my_robot_vision__msg__BucketInfo__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_runtime_c__type_description__TypeSource *
my_robot_vision__msg__BucketInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_vision__msg__BucketInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/BucketInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * my_robot_vision__msg__BucketInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
bool
my_robot_vision__msg__BucketInfo__Sequence__init(my_robot_vision__msg__BucketInfo__Sequence * array, size_t size);

/// Finalize array of msg/BucketInfo messages.
/**
 * It calls
 * my_robot_vision__msg__BucketInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
void
my_robot_vision__msg__BucketInfo__Sequence__fini(my_robot_vision__msg__BucketInfo__Sequence * array);

/// Create array of msg/BucketInfo messages.
/**
 * It allocates the memory for the array and calls
 * my_robot_vision__msg__BucketInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
my_robot_vision__msg__BucketInfo__Sequence *
my_robot_vision__msg__BucketInfo__Sequence__create(size_t size);

/// Destroy array of msg/BucketInfo messages.
/**
 * It calls
 * my_robot_vision__msg__BucketInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
void
my_robot_vision__msg__BucketInfo__Sequence__destroy(my_robot_vision__msg__BucketInfo__Sequence * array);

/// Check for msg/BucketInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
bool
my_robot_vision__msg__BucketInfo__Sequence__are_equal(const my_robot_vision__msg__BucketInfo__Sequence * lhs, const my_robot_vision__msg__BucketInfo__Sequence * rhs);

/// Copy an array of msg/BucketInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
bool
my_robot_vision__msg__BucketInfo__Sequence__copy(
  const my_robot_vision__msg__BucketInfo__Sequence * input,
  my_robot_vision__msg__BucketInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__FUNCTIONS_H_
