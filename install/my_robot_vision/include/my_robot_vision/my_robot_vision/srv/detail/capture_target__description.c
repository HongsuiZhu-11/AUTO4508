// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_robot_vision:srv/CaptureTarget.idl
// generated code does not contain a copyright notice

#include "my_robot_vision/srv/detail/capture_target__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_type_hash_t *
my_robot_vision__srv__CaptureTarget__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x30, 0x45, 0xf7, 0xe8, 0x96, 0x46, 0xe1, 0x5c,
      0xd5, 0x8d, 0x38, 0x73, 0x22, 0x38, 0xd9, 0x30,
      0x82, 0x7d, 0xbc, 0xf1, 0x49, 0x7d, 0x5e, 0x30,
      0x0e, 0x93, 0xb7, 0x42, 0x20, 0x1d, 0x4a, 0x49,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_type_hash_t *
my_robot_vision__srv__CaptureTarget_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x97, 0xe1, 0x2c, 0xa9, 0xd4, 0x7c, 0x90, 0x99,
      0x56, 0xd4, 0xd2, 0x24, 0x50, 0x7e, 0xab, 0x2c,
      0x96, 0x3f, 0x5e, 0xe0, 0x27, 0x94, 0xf4, 0x33,
      0x4d, 0xce, 0x82, 0xc7, 0xe6, 0x10, 0xdc, 0x99,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_type_hash_t *
my_robot_vision__srv__CaptureTarget_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcf, 0x0e, 0x13, 0xfe, 0xc2, 0x50, 0x95, 0x4a,
      0x0c, 0xf5, 0x34, 0x6d, 0x4e, 0x68, 0x8f, 0x8a,
      0x4a, 0x8c, 0xac, 0x1e, 0xa5, 0x86, 0x9c, 0xd6,
      0x1c, 0xd4, 0x5d, 0xc4, 0xf1, 0xbd, 0x1e, 0x6e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_my_robot_vision
const rosidl_type_hash_t *
my_robot_vision__srv__CaptureTarget_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x00, 0xb0, 0xc5, 0xe5, 0xd0, 0x66, 0xef, 0xa5,
      0x6e, 0x09, 0x10, 0xea, 0xcc, 0x11, 0x88, 0xd4,
      0x20, 0x4b, 0x89, 0x14, 0xb2, 0x10, 0xb0, 0x61,
      0xa9, 0x17, 0xd6, 0x70, 0xf9, 0xcc, 0xdd, 0xb5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "my_robot_vision/msg/detail/bucket_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t my_robot_vision__msg__BucketInfo__EXPECTED_HASH = {1, {
    0x53, 0x4d, 0xf5, 0x81, 0x1e, 0x78, 0xa6, 0x53,
    0x8e, 0x35, 0x14, 0xb4, 0x9f, 0x6d, 0x70, 0x90,
    0x95, 0xd0, 0x69, 0x33, 0xe9, 0x94, 0xb9, 0x6d,
    0x75, 0xf5, 0x9b, 0xfb, 0x9c, 0xfa, 0x96, 0xad,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char my_robot_vision__srv__CaptureTarget__TYPE_NAME[] = "my_robot_vision/srv/CaptureTarget";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char my_robot_vision__msg__BucketInfo__TYPE_NAME[] = "my_robot_vision/msg/BucketInfo";
static char my_robot_vision__srv__CaptureTarget_Event__TYPE_NAME[] = "my_robot_vision/srv/CaptureTarget_Event";
static char my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME[] = "my_robot_vision/srv/CaptureTarget_Request";
static char my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME[] = "my_robot_vision/srv/CaptureTarget_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char my_robot_vision__srv__CaptureTarget__FIELD_NAME__request_message[] = "request_message";
static char my_robot_vision__srv__CaptureTarget__FIELD_NAME__response_message[] = "response_message";
static char my_robot_vision__srv__CaptureTarget__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field my_robot_vision__srv__CaptureTarget__FIELDS[] = {
  {
    {my_robot_vision__srv__CaptureTarget__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {my_robot_vision__srv__CaptureTarget_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_robot_vision__srv__CaptureTarget__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__msg__BucketInfo__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_vision__srv__CaptureTarget__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_vision__srv__CaptureTarget__TYPE_NAME, 33, 33},
      {my_robot_vision__srv__CaptureTarget__FIELDS, 3, 3},
    },
    {my_robot_vision__srv__CaptureTarget__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&my_robot_vision__msg__BucketInfo__EXPECTED_HASH, my_robot_vision__msg__BucketInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = my_robot_vision__msg__BucketInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = my_robot_vision__srv__CaptureTarget_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = my_robot_vision__srv__CaptureTarget_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = my_robot_vision__srv__CaptureTarget_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char my_robot_vision__srv__CaptureTarget_Request__FIELD_NAME__robot_x[] = "robot_x";
static char my_robot_vision__srv__CaptureTarget_Request__FIELD_NAME__robot_y[] = "robot_y";
static char my_robot_vision__srv__CaptureTarget_Request__FIELD_NAME__robot_theta_deg[] = "robot_theta_deg";

static rosidl_runtime_c__type_description__Field my_robot_vision__srv__CaptureTarget_Request__FIELDS[] = {
  {
    {my_robot_vision__srv__CaptureTarget_Request__FIELD_NAME__robot_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Request__FIELD_NAME__robot_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Request__FIELD_NAME__robot_theta_deg, 15, 15},
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
my_robot_vision__srv__CaptureTarget_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME, 41, 41},
      {my_robot_vision__srv__CaptureTarget_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__success[] = "success";
static char my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__cone_image_path[] = "cone_image_path";
static char my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__world_x[] = "world_x";
static char my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__world_y[] = "world_y";
static char my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__buckets[] = "buckets";

static rosidl_runtime_c__type_description__Field my_robot_vision__srv__CaptureTarget_Response__FIELDS[] = {
  {
    {my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__cone_image_path, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__world_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__world_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Response__FIELD_NAME__buckets, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {my_robot_vision__msg__BucketInfo__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_robot_vision__srv__CaptureTarget_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {my_robot_vision__msg__BucketInfo__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_vision__srv__CaptureTarget_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME, 42, 42},
      {my_robot_vision__srv__CaptureTarget_Response__FIELDS, 5, 5},
    },
    {my_robot_vision__srv__CaptureTarget_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&my_robot_vision__msg__BucketInfo__EXPECTED_HASH, my_robot_vision__msg__BucketInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = my_robot_vision__msg__BucketInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char my_robot_vision__srv__CaptureTarget_Event__FIELD_NAME__info[] = "info";
static char my_robot_vision__srv__CaptureTarget_Event__FIELD_NAME__request[] = "request";
static char my_robot_vision__srv__CaptureTarget_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field my_robot_vision__srv__CaptureTarget_Event__FIELDS[] = {
  {
    {my_robot_vision__srv__CaptureTarget_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_robot_vision__srv__CaptureTarget_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__msg__BucketInfo__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_vision__srv__CaptureTarget_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_vision__srv__CaptureTarget_Event__TYPE_NAME, 39, 39},
      {my_robot_vision__srv__CaptureTarget_Event__FIELDS, 3, 3},
    },
    {my_robot_vision__srv__CaptureTarget_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&my_robot_vision__msg__BucketInfo__EXPECTED_HASH, my_robot_vision__msg__BucketInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = my_robot_vision__msg__BucketInfo__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = my_robot_vision__srv__CaptureTarget_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = my_robot_vision__srv__CaptureTarget_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 robot_x\n"
  "float64 robot_y\n"
  "float64 robot_theta_deg\n"
  "---\n"
  "\n"
  "bool success\n"
  "string cone_image_path\n"
  "float64 world_x\n"
  "float64 world_y\n"
  "BucketInfo[] buckets";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_robot_vision__srv__CaptureTarget__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_vision__srv__CaptureTarget__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 150, 150},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
my_robot_vision__srv__CaptureTarget_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_vision__srv__CaptureTarget_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
my_robot_vision__srv__CaptureTarget_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_vision__srv__CaptureTarget_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
my_robot_vision__srv__CaptureTarget_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_vision__srv__CaptureTarget_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_vision__srv__CaptureTarget__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_vision__srv__CaptureTarget__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *my_robot_vision__msg__BucketInfo__get_individual_type_description_source(NULL);
    sources[3] = *my_robot_vision__srv__CaptureTarget_Event__get_individual_type_description_source(NULL);
    sources[4] = *my_robot_vision__srv__CaptureTarget_Request__get_individual_type_description_source(NULL);
    sources[5] = *my_robot_vision__srv__CaptureTarget_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_vision__srv__CaptureTarget_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_vision__srv__CaptureTarget_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_vision__srv__CaptureTarget_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_vision__srv__CaptureTarget_Response__get_individual_type_description_source(NULL),
    sources[1] = *my_robot_vision__msg__BucketInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_vision__srv__CaptureTarget_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_vision__srv__CaptureTarget_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *my_robot_vision__msg__BucketInfo__get_individual_type_description_source(NULL);
    sources[3] = *my_robot_vision__srv__CaptureTarget_Request__get_individual_type_description_source(NULL);
    sources[4] = *my_robot_vision__srv__CaptureTarget_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
