// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/msg/bucket_info.hpp"


#ifndef MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__BUILDER_HPP_
#define MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_vision/msg/detail/bucket_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_vision
{

namespace msg
{

namespace builder
{

class Init_BucketInfo_y
{
public:
  explicit Init_BucketInfo_y(::my_robot_vision::msg::BucketInfo & msg)
  : msg_(msg)
  {}
  ::my_robot_vision::msg::BucketInfo y(::my_robot_vision::msg::BucketInfo::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_vision::msg::BucketInfo msg_;
};

class Init_BucketInfo_x
{
public:
  explicit Init_BucketInfo_x(::my_robot_vision::msg::BucketInfo & msg)
  : msg_(msg)
  {}
  Init_BucketInfo_y x(::my_robot_vision::msg::BucketInfo::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_BucketInfo_y(msg_);
  }

private:
  ::my_robot_vision::msg::BucketInfo msg_;
};

class Init_BucketInfo_distance_to_cone
{
public:
  explicit Init_BucketInfo_distance_to_cone(::my_robot_vision::msg::BucketInfo & msg)
  : msg_(msg)
  {}
  Init_BucketInfo_x distance_to_cone(::my_robot_vision::msg::BucketInfo::_distance_to_cone_type arg)
  {
    msg_.distance_to_cone = std::move(arg);
    return Init_BucketInfo_x(msg_);
  }

private:
  ::my_robot_vision::msg::BucketInfo msg_;
};

class Init_BucketInfo_image_path
{
public:
  explicit Init_BucketInfo_image_path(::my_robot_vision::msg::BucketInfo & msg)
  : msg_(msg)
  {}
  Init_BucketInfo_distance_to_cone image_path(::my_robot_vision::msg::BucketInfo::_image_path_type arg)
  {
    msg_.image_path = std::move(arg);
    return Init_BucketInfo_distance_to_cone(msg_);
  }

private:
  ::my_robot_vision::msg::BucketInfo msg_;
};

class Init_BucketInfo_color
{
public:
  Init_BucketInfo_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BucketInfo_image_path color(::my_robot_vision::msg::BucketInfo::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_BucketInfo_image_path(msg_);
  }

private:
  ::my_robot_vision::msg::BucketInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_vision::msg::BucketInfo>()
{
  return my_robot_vision::msg::builder::Init_BucketInfo_color();
}

}  // namespace my_robot_vision

#endif  // MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__BUILDER_HPP_
