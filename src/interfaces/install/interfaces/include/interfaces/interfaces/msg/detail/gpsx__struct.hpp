// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Gpsx.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/gpsx.hpp"


#ifndef INTERFACES__MSG__DETAIL__GPSX__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__GPSX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Gpsx __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Gpsx __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gpsx_
{
  using Type = Gpsx_<ContainerAllocator>;

  explicit Gpsx_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitude = 0.0;
      this->latitude = 0.0;
      this->altitude = 0.0;
      this->ground_speed = 0.0f;
      this->satellites = 0;
      this->mode_indicator = 0;
      this->separation = 0.0f;
      this->true_course = 0.0f;
      this->true_course_magnetic = 0.0f;
      this->dilution = 0.0f;
      this->utc_time = 0ul;
    }
  }

  explicit Gpsx_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitude = 0.0;
      this->latitude = 0.0;
      this->altitude = 0.0;
      this->ground_speed = 0.0f;
      this->satellites = 0;
      this->mode_indicator = 0;
      this->separation = 0.0f;
      this->true_course = 0.0f;
      this->true_course_magnetic = 0.0f;
      this->dilution = 0.0f;
      this->utc_time = 0ul;
    }
  }

  // field types and members
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _ground_speed_type =
    float;
  _ground_speed_type ground_speed;
  using _satellites_type =
    uint8_t;
  _satellites_type satellites;
  using _mode_indicator_type =
    uint8_t;
  _mode_indicator_type mode_indicator;
  using _separation_type =
    float;
  _separation_type separation;
  using _true_course_type =
    float;
  _true_course_type true_course;
  using _true_course_magnetic_type =
    float;
  _true_course_magnetic_type true_course_magnetic;
  using _dilution_type =
    float;
  _dilution_type dilution;
  using _utc_time_type =
    uint32_t;
  _utc_time_type utc_time;

  // setters for named parameter idiom
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__ground_speed(
    const float & _arg)
  {
    this->ground_speed = _arg;
    return *this;
  }
  Type & set__satellites(
    const uint8_t & _arg)
  {
    this->satellites = _arg;
    return *this;
  }
  Type & set__mode_indicator(
    const uint8_t & _arg)
  {
    this->mode_indicator = _arg;
    return *this;
  }
  Type & set__separation(
    const float & _arg)
  {
    this->separation = _arg;
    return *this;
  }
  Type & set__true_course(
    const float & _arg)
  {
    this->true_course = _arg;
    return *this;
  }
  Type & set__true_course_magnetic(
    const float & _arg)
  {
    this->true_course_magnetic = _arg;
    return *this;
  }
  Type & set__dilution(
    const float & _arg)
  {
    this->dilution = _arg;
    return *this;
  }
  Type & set__utc_time(
    const uint32_t & _arg)
  {
    this->utc_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Gpsx_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Gpsx_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Gpsx_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Gpsx_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Gpsx_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Gpsx_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Gpsx_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Gpsx_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Gpsx_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Gpsx_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Gpsx
    std::shared_ptr<interfaces::msg::Gpsx_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Gpsx
    std::shared_ptr<interfaces::msg::Gpsx_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gpsx_ & other) const
  {
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->ground_speed != other.ground_speed) {
      return false;
    }
    if (this->satellites != other.satellites) {
      return false;
    }
    if (this->mode_indicator != other.mode_indicator) {
      return false;
    }
    if (this->separation != other.separation) {
      return false;
    }
    if (this->true_course != other.true_course) {
      return false;
    }
    if (this->true_course_magnetic != other.true_course_magnetic) {
      return false;
    }
    if (this->dilution != other.dilution) {
      return false;
    }
    if (this->utc_time != other.utc_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gpsx_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gpsx_

// alias to use template instance with default allocator
using Gpsx =
  interfaces::msg::Gpsx_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__GPSX__STRUCT_HPP_
