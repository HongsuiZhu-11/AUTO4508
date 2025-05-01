// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_vision:srv/CaptureTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/srv/capture_target.hpp"


#ifndef MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__BUILDER_HPP_
#define MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_vision/srv/detail/capture_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_vision
{

namespace srv
{

namespace builder
{

class Init_CaptureTarget_Request_robot_theta_deg
{
public:
  explicit Init_CaptureTarget_Request_robot_theta_deg(::my_robot_vision::srv::CaptureTarget_Request & msg)
  : msg_(msg)
  {}
  ::my_robot_vision::srv::CaptureTarget_Request robot_theta_deg(::my_robot_vision::srv::CaptureTarget_Request::_robot_theta_deg_type arg)
  {
    msg_.robot_theta_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Request msg_;
};

class Init_CaptureTarget_Request_robot_y
{
public:
  explicit Init_CaptureTarget_Request_robot_y(::my_robot_vision::srv::CaptureTarget_Request & msg)
  : msg_(msg)
  {}
  Init_CaptureTarget_Request_robot_theta_deg robot_y(::my_robot_vision::srv::CaptureTarget_Request::_robot_y_type arg)
  {
    msg_.robot_y = std::move(arg);
    return Init_CaptureTarget_Request_robot_theta_deg(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Request msg_;
};

class Init_CaptureTarget_Request_robot_x
{
public:
  Init_CaptureTarget_Request_robot_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureTarget_Request_robot_y robot_x(::my_robot_vision::srv::CaptureTarget_Request::_robot_x_type arg)
  {
    msg_.robot_x = std::move(arg);
    return Init_CaptureTarget_Request_robot_y(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_vision::srv::CaptureTarget_Request>()
{
  return my_robot_vision::srv::builder::Init_CaptureTarget_Request_robot_x();
}

}  // namespace my_robot_vision


namespace my_robot_vision
{

namespace srv
{

namespace builder
{

class Init_CaptureTarget_Response_buckets
{
public:
  explicit Init_CaptureTarget_Response_buckets(::my_robot_vision::srv::CaptureTarget_Response & msg)
  : msg_(msg)
  {}
  ::my_robot_vision::srv::CaptureTarget_Response buckets(::my_robot_vision::srv::CaptureTarget_Response::_buckets_type arg)
  {
    msg_.buckets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Response msg_;
};

class Init_CaptureTarget_Response_world_y
{
public:
  explicit Init_CaptureTarget_Response_world_y(::my_robot_vision::srv::CaptureTarget_Response & msg)
  : msg_(msg)
  {}
  Init_CaptureTarget_Response_buckets world_y(::my_robot_vision::srv::CaptureTarget_Response::_world_y_type arg)
  {
    msg_.world_y = std::move(arg);
    return Init_CaptureTarget_Response_buckets(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Response msg_;
};

class Init_CaptureTarget_Response_world_x
{
public:
  explicit Init_CaptureTarget_Response_world_x(::my_robot_vision::srv::CaptureTarget_Response & msg)
  : msg_(msg)
  {}
  Init_CaptureTarget_Response_world_y world_x(::my_robot_vision::srv::CaptureTarget_Response::_world_x_type arg)
  {
    msg_.world_x = std::move(arg);
    return Init_CaptureTarget_Response_world_y(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Response msg_;
};

class Init_CaptureTarget_Response_cone_image_path
{
public:
  explicit Init_CaptureTarget_Response_cone_image_path(::my_robot_vision::srv::CaptureTarget_Response & msg)
  : msg_(msg)
  {}
  Init_CaptureTarget_Response_world_x cone_image_path(::my_robot_vision::srv::CaptureTarget_Response::_cone_image_path_type arg)
  {
    msg_.cone_image_path = std::move(arg);
    return Init_CaptureTarget_Response_world_x(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Response msg_;
};

class Init_CaptureTarget_Response_success
{
public:
  Init_CaptureTarget_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureTarget_Response_cone_image_path success(::my_robot_vision::srv::CaptureTarget_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CaptureTarget_Response_cone_image_path(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_vision::srv::CaptureTarget_Response>()
{
  return my_robot_vision::srv::builder::Init_CaptureTarget_Response_success();
}

}  // namespace my_robot_vision


namespace my_robot_vision
{

namespace srv
{

namespace builder
{

class Init_CaptureTarget_Event_response
{
public:
  explicit Init_CaptureTarget_Event_response(::my_robot_vision::srv::CaptureTarget_Event & msg)
  : msg_(msg)
  {}
  ::my_robot_vision::srv::CaptureTarget_Event response(::my_robot_vision::srv::CaptureTarget_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Event msg_;
};

class Init_CaptureTarget_Event_request
{
public:
  explicit Init_CaptureTarget_Event_request(::my_robot_vision::srv::CaptureTarget_Event & msg)
  : msg_(msg)
  {}
  Init_CaptureTarget_Event_response request(::my_robot_vision::srv::CaptureTarget_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CaptureTarget_Event_response(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Event msg_;
};

class Init_CaptureTarget_Event_info
{
public:
  Init_CaptureTarget_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureTarget_Event_request info(::my_robot_vision::srv::CaptureTarget_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CaptureTarget_Event_request(msg_);
  }

private:
  ::my_robot_vision::srv::CaptureTarget_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_vision::srv::CaptureTarget_Event>()
{
  return my_robot_vision::srv::builder::Init_CaptureTarget_Event_info();
}

}  // namespace my_robot_vision

#endif  // MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__BUILDER_HPP_
