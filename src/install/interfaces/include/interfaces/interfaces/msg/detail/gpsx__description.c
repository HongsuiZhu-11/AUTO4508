// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/Gpsx.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/gpsx__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__Gpsx__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf5, 0x8f, 0xcc, 0x39, 0x27, 0x86, 0x46, 0x73,
      0xd9, 0x4a, 0xa7, 0x90, 0x43, 0x04, 0x66, 0x5d,
      0x73, 0x0c, 0xcb, 0xbc, 0xd1, 0x3c, 0x8e, 0xf6,
      0xd8, 0x5f, 0x4c, 0x31, 0xa7, 0x34, 0x70, 0xa8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__Gpsx__TYPE_NAME[] = "interfaces/msg/Gpsx";

// Define type names, field names, and default values
static char interfaces__msg__Gpsx__FIELD_NAME__longitude[] = "longitude";
static char interfaces__msg__Gpsx__FIELD_NAME__latitude[] = "latitude";
static char interfaces__msg__Gpsx__FIELD_NAME__altitude[] = "altitude";
static char interfaces__msg__Gpsx__FIELD_NAME__ground_speed[] = "ground_speed";
static char interfaces__msg__Gpsx__FIELD_NAME__satellites[] = "satellites";
static char interfaces__msg__Gpsx__FIELD_NAME__mode_indicator[] = "mode_indicator";
static char interfaces__msg__Gpsx__FIELD_NAME__separation[] = "separation";
static char interfaces__msg__Gpsx__FIELD_NAME__true_course[] = "true_course";
static char interfaces__msg__Gpsx__FIELD_NAME__true_course_magnetic[] = "true_course_magnetic";
static char interfaces__msg__Gpsx__FIELD_NAME__dilution[] = "dilution";
static char interfaces__msg__Gpsx__FIELD_NAME__utc_time[] = "utc_time";

static rosidl_runtime_c__type_description__Field interfaces__msg__Gpsx__FIELDS[] = {
  {
    {interfaces__msg__Gpsx__FIELD_NAME__longitude, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__latitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__ground_speed, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__satellites, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__mode_indicator, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__separation, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__true_course, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__true_course_magnetic, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__dilution, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__Gpsx__FIELD_NAME__utc_time, 8, 8},
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
interfaces__msg__Gpsx__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__Gpsx__TYPE_NAME, 19, 19},
      {interfaces__msg__Gpsx__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 longitude\n"
  "float64 latitude\n"
  "float64 altitude\n"
  "float32 ground_speed\n"
  "uint8 satellites\n"
  "uint8 mode_indicator\n"
  "float32 separation\n"
  "float32 true_course\n"
  "float32 true_course_magnetic\n"
  "float32 dilution\n"
  "uint32 utc_time";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__Gpsx__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__Gpsx__TYPE_NAME, 19, 19},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 211, 211},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__Gpsx__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__Gpsx__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
