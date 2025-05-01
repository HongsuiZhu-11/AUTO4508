// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/msg/bucket_info.hpp"


#ifndef MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__TRAITS_HPP_
#define MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_vision/msg/detail/bucket_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_vision
{

namespace msg
{

inline void to_flow_style_yaml(
  const BucketInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << ", ";
  }

  // member: image_path
  {
    out << "image_path: ";
    rosidl_generator_traits::value_to_yaml(msg.image_path, out);
    out << ", ";
  }

  // member: distance_to_cone
  {
    out << "distance_to_cone: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_cone, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BucketInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }

  // member: image_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image_path: ";
    rosidl_generator_traits::value_to_yaml(msg.image_path, out);
    out << "\n";
  }

  // member: distance_to_cone
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_to_cone: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_to_cone, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BucketInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace my_robot_vision

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_vision::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_vision::msg::BucketInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_vision::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_vision::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_vision::msg::BucketInfo & msg)
{
  return my_robot_vision::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_vision::msg::BucketInfo>()
{
  return "my_robot_vision::msg::BucketInfo";
}

template<>
inline const char * name<my_robot_vision::msg::BucketInfo>()
{
  return "my_robot_vision/msg/BucketInfo";
}

template<>
struct has_fixed_size<my_robot_vision::msg::BucketInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_vision::msg::BucketInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_robot_vision::msg::BucketInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__TRAITS_HPP_
