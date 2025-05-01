// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from my_robot_vision:srv/CaptureTarget.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "my_robot_vision/srv/detail/capture_target__functions.h"
#include "my_robot_vision/srv/detail/capture_target__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace my_robot_vision
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CaptureTarget_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_robot_vision::srv::CaptureTarget_Request(_init);
}

void CaptureTarget_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_robot_vision::srv::CaptureTarget_Request *>(message_memory);
  typed_message->~CaptureTarget_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CaptureTarget_Request_message_member_array[3] = {
  {
    "robot_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Request, robot_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Request, robot_y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_theta_deg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Request, robot_theta_deg),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CaptureTarget_Request_message_members = {
  "my_robot_vision::srv",  // message namespace
  "CaptureTarget_Request",  // message name
  3,  // number of fields
  sizeof(my_robot_vision::srv::CaptureTarget_Request),
  false,  // has_any_key_member_
  CaptureTarget_Request_message_member_array,  // message members
  CaptureTarget_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CaptureTarget_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CaptureTarget_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CaptureTarget_Request_message_members,
  get_message_typesupport_handle_function,
  &my_robot_vision__srv__CaptureTarget_Request__get_type_hash,
  &my_robot_vision__srv__CaptureTarget_Request__get_type_description,
  &my_robot_vision__srv__CaptureTarget_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace my_robot_vision


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Request>()
{
  return &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_robot_vision, srv, CaptureTarget_Request)() {
  return &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "my_robot_vision/srv/detail/capture_target__functions.h"
// already included above
// #include "my_robot_vision/srv/detail/capture_target__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace my_robot_vision
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CaptureTarget_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_robot_vision::srv::CaptureTarget_Response(_init);
}

void CaptureTarget_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_robot_vision::srv::CaptureTarget_Response *>(message_memory);
  typed_message->~CaptureTarget_Response();
}

size_t size_function__CaptureTarget_Response__buckets(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<my_robot_vision::msg::BucketInfo> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CaptureTarget_Response__buckets(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<my_robot_vision::msg::BucketInfo> *>(untyped_member);
  return &member[index];
}

void * get_function__CaptureTarget_Response__buckets(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<my_robot_vision::msg::BucketInfo> *>(untyped_member);
  return &member[index];
}

void fetch_function__CaptureTarget_Response__buckets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const my_robot_vision::msg::BucketInfo *>(
    get_const_function__CaptureTarget_Response__buckets(untyped_member, index));
  auto & value = *reinterpret_cast<my_robot_vision::msg::BucketInfo *>(untyped_value);
  value = item;
}

void assign_function__CaptureTarget_Response__buckets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<my_robot_vision::msg::BucketInfo *>(
    get_function__CaptureTarget_Response__buckets(untyped_member, index));
  const auto & value = *reinterpret_cast<const my_robot_vision::msg::BucketInfo *>(untyped_value);
  item = value;
}

void resize_function__CaptureTarget_Response__buckets(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<my_robot_vision::msg::BucketInfo> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CaptureTarget_Response_message_member_array[5] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cone_image_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Response, cone_image_path),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "world_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Response, world_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "world_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Response, world_y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "buckets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_robot_vision::msg::BucketInfo>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Response, buckets),  // bytes offset in struct
    nullptr,  // default value
    size_function__CaptureTarget_Response__buckets,  // size() function pointer
    get_const_function__CaptureTarget_Response__buckets,  // get_const(index) function pointer
    get_function__CaptureTarget_Response__buckets,  // get(index) function pointer
    fetch_function__CaptureTarget_Response__buckets,  // fetch(index, &value) function pointer
    assign_function__CaptureTarget_Response__buckets,  // assign(index, value) function pointer
    resize_function__CaptureTarget_Response__buckets  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CaptureTarget_Response_message_members = {
  "my_robot_vision::srv",  // message namespace
  "CaptureTarget_Response",  // message name
  5,  // number of fields
  sizeof(my_robot_vision::srv::CaptureTarget_Response),
  false,  // has_any_key_member_
  CaptureTarget_Response_message_member_array,  // message members
  CaptureTarget_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CaptureTarget_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CaptureTarget_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CaptureTarget_Response_message_members,
  get_message_typesupport_handle_function,
  &my_robot_vision__srv__CaptureTarget_Response__get_type_hash,
  &my_robot_vision__srv__CaptureTarget_Response__get_type_description,
  &my_robot_vision__srv__CaptureTarget_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace my_robot_vision


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Response>()
{
  return &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_robot_vision, srv, CaptureTarget_Response)() {
  return &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "my_robot_vision/srv/detail/capture_target__functions.h"
// already included above
// #include "my_robot_vision/srv/detail/capture_target__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace my_robot_vision
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CaptureTarget_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_robot_vision::srv::CaptureTarget_Event(_init);
}

void CaptureTarget_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_robot_vision::srv::CaptureTarget_Event *>(message_memory);
  typed_message->~CaptureTarget_Event();
}

size_t size_function__CaptureTarget_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<my_robot_vision::srv::CaptureTarget_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CaptureTarget_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<my_robot_vision::srv::CaptureTarget_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__CaptureTarget_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<my_robot_vision::srv::CaptureTarget_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__CaptureTarget_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const my_robot_vision::srv::CaptureTarget_Request *>(
    get_const_function__CaptureTarget_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<my_robot_vision::srv::CaptureTarget_Request *>(untyped_value);
  value = item;
}

void assign_function__CaptureTarget_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<my_robot_vision::srv::CaptureTarget_Request *>(
    get_function__CaptureTarget_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const my_robot_vision::srv::CaptureTarget_Request *>(untyped_value);
  item = value;
}

void resize_function__CaptureTarget_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<my_robot_vision::srv::CaptureTarget_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CaptureTarget_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<my_robot_vision::srv::CaptureTarget_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CaptureTarget_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<my_robot_vision::srv::CaptureTarget_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__CaptureTarget_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<my_robot_vision::srv::CaptureTarget_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__CaptureTarget_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const my_robot_vision::srv::CaptureTarget_Response *>(
    get_const_function__CaptureTarget_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<my_robot_vision::srv::CaptureTarget_Response *>(untyped_value);
  value = item;
}

void assign_function__CaptureTarget_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<my_robot_vision::srv::CaptureTarget_Response *>(
    get_function__CaptureTarget_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const my_robot_vision::srv::CaptureTarget_Response *>(untyped_value);
  item = value;
}

void resize_function__CaptureTarget_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<my_robot_vision::srv::CaptureTarget_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CaptureTarget_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__CaptureTarget_Event__request,  // size() function pointer
    get_const_function__CaptureTarget_Event__request,  // get_const(index) function pointer
    get_function__CaptureTarget_Event__request,  // get(index) function pointer
    fetch_function__CaptureTarget_Event__request,  // fetch(index, &value) function pointer
    assign_function__CaptureTarget_Event__request,  // assign(index, value) function pointer
    resize_function__CaptureTarget_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(my_robot_vision::srv::CaptureTarget_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__CaptureTarget_Event__response,  // size() function pointer
    get_const_function__CaptureTarget_Event__response,  // get_const(index) function pointer
    get_function__CaptureTarget_Event__response,  // get(index) function pointer
    fetch_function__CaptureTarget_Event__response,  // fetch(index, &value) function pointer
    assign_function__CaptureTarget_Event__response,  // assign(index, value) function pointer
    resize_function__CaptureTarget_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CaptureTarget_Event_message_members = {
  "my_robot_vision::srv",  // message namespace
  "CaptureTarget_Event",  // message name
  3,  // number of fields
  sizeof(my_robot_vision::srv::CaptureTarget_Event),
  false,  // has_any_key_member_
  CaptureTarget_Event_message_member_array,  // message members
  CaptureTarget_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  CaptureTarget_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CaptureTarget_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CaptureTarget_Event_message_members,
  get_message_typesupport_handle_function,
  &my_robot_vision__srv__CaptureTarget_Event__get_type_hash,
  &my_robot_vision__srv__CaptureTarget_Event__get_type_description,
  &my_robot_vision__srv__CaptureTarget_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace my_robot_vision


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Event>()
{
  return &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_robot_vision, srv, CaptureTarget_Event)() {
  return &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "my_robot_vision/srv/detail/capture_target__functions.h"
// already included above
// #include "my_robot_vision/srv/detail/capture_target__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace my_robot_vision
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers CaptureTarget_service_members = {
  "my_robot_vision::srv",  // service namespace
  "CaptureTarget",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<my_robot_vision::srv::CaptureTarget>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t CaptureTarget_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CaptureTarget_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<my_robot_vision::srv::CaptureTarget_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<my_robot_vision::srv::CaptureTarget>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<my_robot_vision::srv::CaptureTarget>,
  &my_robot_vision__srv__CaptureTarget__get_type_hash,
  &my_robot_vision__srv__CaptureTarget__get_type_description,
  &my_robot_vision__srv__CaptureTarget__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace my_robot_vision


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<my_robot_vision::srv::CaptureTarget>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::my_robot_vision::srv::rosidl_typesupport_introspection_cpp::CaptureTarget_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::my_robot_vision::srv::CaptureTarget_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::my_robot_vision::srv::CaptureTarget_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::my_robot_vision::srv::CaptureTarget_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_robot_vision, srv, CaptureTarget)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<my_robot_vision::srv::CaptureTarget>();
}

#ifdef __cplusplus
}
#endif
