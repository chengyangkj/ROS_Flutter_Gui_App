// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topology_msgs:srv/NavToTopologyPoint.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__TRAITS_HPP_
#define TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topology_msgs/srv/detail/nav_to_topology_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace topology_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const NavToTopologyPoint_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: point_name
  {
    out << "point_name: ";
    rosidl_generator_traits::value_to_yaml(msg.point_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavToTopologyPoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: point_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_name: ";
    rosidl_generator_traits::value_to_yaml(msg.point_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavToTopologyPoint_Request & msg, bool use_flow_style = false)
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

}  // namespace topology_msgs

namespace rosidl_generator_traits
{

[[deprecated("use topology_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const topology_msgs::srv::NavToTopologyPoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::srv::NavToTopologyPoint_Request & msg)
{
  return topology_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::srv::NavToTopologyPoint_Request>()
{
  return "topology_msgs::srv::NavToTopologyPoint_Request";
}

template<>
inline const char * name<topology_msgs::srv::NavToTopologyPoint_Request>()
{
  return "topology_msgs/srv/NavToTopologyPoint_Request";
}

template<>
struct has_fixed_size<topology_msgs::srv::NavToTopologyPoint_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::srv::NavToTopologyPoint_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::srv::NavToTopologyPoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace topology_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const NavToTopologyPoint_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_success
  {
    out << "is_success: ";
    rosidl_generator_traits::value_to_yaml(msg.is_success, out);
    out << ", ";
  }

  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: msg
  {
    out << "msg: ";
    rosidl_generator_traits::value_to_yaml(msg.msg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavToTopologyPoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_success: ";
    rosidl_generator_traits::value_to_yaml(msg.is_success, out);
    out << "\n";
  }

  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: msg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msg: ";
    rosidl_generator_traits::value_to_yaml(msg.msg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavToTopologyPoint_Response & msg, bool use_flow_style = false)
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

}  // namespace topology_msgs

namespace rosidl_generator_traits
{

[[deprecated("use topology_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const topology_msgs::srv::NavToTopologyPoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::srv::NavToTopologyPoint_Response & msg)
{
  return topology_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::srv::NavToTopologyPoint_Response>()
{
  return "topology_msgs::srv::NavToTopologyPoint_Response";
}

template<>
inline const char * name<topology_msgs::srv::NavToTopologyPoint_Response>()
{
  return "topology_msgs/srv/NavToTopologyPoint_Response";
}

template<>
struct has_fixed_size<topology_msgs::srv::NavToTopologyPoint_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::srv::NavToTopologyPoint_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::srv::NavToTopologyPoint_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<topology_msgs::srv::NavToTopologyPoint>()
{
  return "topology_msgs::srv::NavToTopologyPoint";
}

template<>
inline const char * name<topology_msgs::srv::NavToTopologyPoint>()
{
  return "topology_msgs/srv/NavToTopologyPoint";
}

template<>
struct has_fixed_size<topology_msgs::srv::NavToTopologyPoint>
  : std::integral_constant<
    bool,
    has_fixed_size<topology_msgs::srv::NavToTopologyPoint_Request>::value &&
    has_fixed_size<topology_msgs::srv::NavToTopologyPoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<topology_msgs::srv::NavToTopologyPoint>
  : std::integral_constant<
    bool,
    has_bounded_size<topology_msgs::srv::NavToTopologyPoint_Request>::value &&
    has_bounded_size<topology_msgs::srv::NavToTopologyPoint_Response>::value
  >
{
};

template<>
struct is_service<topology_msgs::srv::NavToTopologyPoint>
  : std::true_type
{
};

template<>
struct is_service_request<topology_msgs::srv::NavToTopologyPoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<topology_msgs::srv::NavToTopologyPoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__TRAITS_HPP_
