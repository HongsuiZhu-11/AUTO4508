// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_robot_vision:msg/BucketInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/msg/bucket_info.hpp"


#ifndef MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__STRUCT_HPP_
#define MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_robot_vision__msg__BucketInfo __attribute__((deprecated))
#else
# define DEPRECATED__my_robot_vision__msg__BucketInfo __declspec(deprecated)
#endif

namespace my_robot_vision
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BucketInfo_
{
  using Type = BucketInfo_<ContainerAllocator>;

  explicit BucketInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color = "";
      this->image_path = "";
      this->distance_to_cone = 0.0;
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit BucketInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc),
    image_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color = "";
      this->image_path = "";
      this->distance_to_cone = 0.0;
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _color_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_type color;
  using _image_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _image_path_type image_path;
  using _distance_to_cone_type =
    double;
  _distance_to_cone_type distance_to_cone;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__color(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__image_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->image_path = _arg;
    return *this;
  }
  Type & set__distance_to_cone(
    const double & _arg)
  {
    this->distance_to_cone = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_robot_vision::msg::BucketInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_robot_vision::msg::BucketInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_robot_vision::msg::BucketInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_robot_vision::msg::BucketInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_robot_vision__msg__BucketInfo
    std::shared_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_robot_vision__msg__BucketInfo
    std::shared_ptr<my_robot_vision::msg::BucketInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BucketInfo_ & other) const
  {
    if (this->color != other.color) {
      return false;
    }
    if (this->image_path != other.image_path) {
      return false;
    }
    if (this->distance_to_cone != other.distance_to_cone) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const BucketInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BucketInfo_

// alias to use template instance with default allocator
using BucketInfo =
  my_robot_vision::msg::BucketInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_robot_vision

#endif  // MY_ROBOT_VISION__MSG__DETAIL__BUCKET_INFO__STRUCT_HPP_
