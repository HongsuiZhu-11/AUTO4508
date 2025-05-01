// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_vision:srv/CaptureTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_vision/srv/capture_target.hpp"


#ifndef MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__TRAITS_HPP_
#define MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_vision/srv/detail/capture_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_vision
{

namespace srv
{

inline void to_flow_style_yaml(
  const CaptureTarget_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_x
  {
    out << "robot_x: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_x, out);
    out << ", ";
  }

  // member: robot_y
  {
    out << "robot_y: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_y, out);
    out << ", ";
  }

  // member: robot_theta_deg
  {
    out << "robot_theta_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_theta_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CaptureTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_x: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_x, out);
    out << "\n";
  }

  // member: robot_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_y: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_y, out);
    out << "\n";
  }

  // member: robot_theta_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_theta_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_theta_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CaptureTarget_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_vision

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_vision::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_vision::srv::CaptureTarget_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_vision::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_vision::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_vision::srv::CaptureTarget_Request & msg)
{
  return my_robot_vision::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_vision::srv::CaptureTarget_Request>()
{
  return "my_robot_vision::srv::CaptureTarget_Request";
}

template<>
inline const char * name<my_robot_vision::srv::CaptureTarget_Request>()
{
  return "my_robot_vision/srv/CaptureTarget_Request";
}

template<>
struct has_fixed_size<my_robot_vision::srv::CaptureTarget_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_vision::srv::CaptureTarget_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_vision::srv::CaptureTarget_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'buckets'
#include "my_robot_vision/msg/detail/bucket_info__traits.hpp"

namespace my_robot_vision
{

namespace srv
{

inline void to_flow_style_yaml(
  const CaptureTarget_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: cone_image_path
  {
    out << "cone_image_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cone_image_path, out);
    out << ", ";
  }

  // member: world_x
  {
    out << "world_x: ";
    rosidl_generator_traits::value_to_yaml(msg.world_x, out);
    out << ", ";
  }

  // member: world_y
  {
    out << "world_y: ";
    rosidl_generator_traits::value_to_yaml(msg.world_y, out);
    out << ", ";
  }

  // member: buckets
  {
    if (msg.buckets.size() == 0) {
      out << "buckets: []";
    } else {
      out << "buckets: [";
      size_t pending_items = msg.buckets.size();
      for (auto item : msg.buckets) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CaptureTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: cone_image_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cone_image_path: ";
    rosidl_generator_traits::value_to_yaml(msg.cone_image_path, out);
    out << "\n";
  }

  // member: world_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_x: ";
    rosidl_generator_traits::value_to_yaml(msg.world_x, out);
    out << "\n";
  }

  // member: world_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_y: ";
    rosidl_generator_traits::value_to_yaml(msg.world_y, out);
    out << "\n";
  }

  // member: buckets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.buckets.size() == 0) {
      out << "buckets: []\n";
    } else {
      out << "buckets:\n";
      for (auto item : msg.buckets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CaptureTarget_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_vision

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_vision::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_vision::srv::CaptureTarget_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_vision::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_vision::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_vision::srv::CaptureTarget_Response & msg)
{
  return my_robot_vision::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_vision::srv::CaptureTarget_Response>()
{
  return "my_robot_vision::srv::CaptureTarget_Response";
}

template<>
inline const char * name<my_robot_vision::srv::CaptureTarget_Response>()
{
  return "my_robot_vision/srv/CaptureTarget_Response";
}

template<>
struct has_fixed_size<my_robot_vision::srv::CaptureTarget_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_vision::srv::CaptureTarget_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_robot_vision::srv::CaptureTarget_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace my_robot_vision
{

namespace srv
{

inline void to_flow_style_yaml(
  const CaptureTarget_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CaptureTarget_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CaptureTarget_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_vision

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_vision::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_vision::srv::CaptureTarget_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_vision::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_vision::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_vision::srv::CaptureTarget_Event & msg)
{
  return my_robot_vision::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_vision::srv::CaptureTarget_Event>()
{
  return "my_robot_vision::srv::CaptureTarget_Event";
}

template<>
inline const char * name<my_robot_vision::srv::CaptureTarget_Event>()
{
  return "my_robot_vision/srv/CaptureTarget_Event";
}

template<>
struct has_fixed_size<my_robot_vision::srv::CaptureTarget_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_vision::srv::CaptureTarget_Event>
  : std::integral_constant<bool, has_bounded_size<my_robot_vision::srv::CaptureTarget_Request>::value && has_bounded_size<my_robot_vision::srv::CaptureTarget_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<my_robot_vision::srv::CaptureTarget_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_robot_vision::srv::CaptureTarget>()
{
  return "my_robot_vision::srv::CaptureTarget";
}

template<>
inline const char * name<my_robot_vision::srv::CaptureTarget>()
{
  return "my_robot_vision/srv/CaptureTarget";
}

template<>
struct has_fixed_size<my_robot_vision::srv::CaptureTarget>
  : std::integral_constant<
    bool,
    has_fixed_size<my_robot_vision::srv::CaptureTarget_Request>::value &&
    has_fixed_size<my_robot_vision::srv::CaptureTarget_Response>::value
  >
{
};

template<>
struct has_bounded_size<my_robot_vision::srv::CaptureTarget>
  : std::integral_constant<
    bool,
    has_bounded_size<my_robot_vision::srv::CaptureTarget_Request>::value &&
    has_bounded_size<my_robot_vision::srv::CaptureTarget_Response>::value
  >
{
};

template<>
struct is_service<my_robot_vision::srv::CaptureTarget>
  : std::true_type
{
};

template<>
struct is_service_request<my_robot_vision::srv::CaptureTarget_Request>
  : std::true_type
{
};

template<>
struct is_service_response<my_robot_vision::srv::CaptureTarget_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_VISION__SRV__DETAIL__CAPTURE_TARGET__TRAITS_HPP_
