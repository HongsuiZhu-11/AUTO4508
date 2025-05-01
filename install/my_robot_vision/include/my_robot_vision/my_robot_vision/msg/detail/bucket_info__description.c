// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice

#include "my_robot_vision/msg/detail/bucket_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_type_hash_t *
my_robot_vision__msg__BucketInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x53, 0x4d, 0xf5, 0x81, 0x1e, 0x78, 0xa6, 0x53,
      0x8e, 0x35, 0x14, 0xb4, 0x9f, 0x6d, 0x70, 0x90,
      0x95, 0xd0, 0x69, 0x33, 0xe9, 0x94, 0xb9, 0x6d,
      0x75, 0xf5, 0x9b, 0xfb, 0x9c, 0xfa, 0x96, 0xad,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_robot_vision__msg__BucketInfo__TYPE_NAME[] = "my_robot_vision/msg/BucketInfo";

// Define type names, field names, and default values
static char my_robot_vision__msg__BucketInfo__FIELD_NAME__color[] = "color";
static char my_robot_vision__msg__BucketInfo__FIELD_NAME__image_path[] = "image_path";
static char my_robot_vision__msg__BucketInfo__FIELD_NAME__distance_to_cone[] = "distance_to_cone";
static char my_robot_vision__msg__BucketInfo__FIELD_NAME__x[] = "x";
static char my_robot_vision__msg__BucketInfo__FIELD_NAME__y[] = "y";

static rosidl_runtime_c__type_description__Field my_robot_vision__msg__BucketInfo__FIELDS[] = {
  {
    {my_robot_vision__msg__BucketInfo__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__msg__BucketInfo__FIELD_NAME__image_path, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__msg__BucketInfo__FIELD_NAME__distance_to_cone, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__msg__BucketInfo__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__msg__BucketInfo__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_vision__msg__BucketInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_vision__msg__BucketInfo__TYPE_NAME, 30, 30},
      {my_robot_vision__msg__BucketInfo__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string color\n"
  "string image_path\n"
  "float64 distance_to_cone\n"
  "float64 x\n"
  "float64 y";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_robot_vision__msg__BucketInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_vision__msg__BucketInfo__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 75, 75},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_vision__msg__BucketInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_vision__msg__BucketInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
