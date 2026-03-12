// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topology_msgs:msg/RouteConnection.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__TRAITS_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topology_msgs/msg/detail/route_connection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'route_info'
#include "topology_msgs/msg/detail/route_info__traits.hpp"

namespace topology_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RouteConnection & msg,
  std::ostream & out)
{
  out << "{";
  // member: from_point
  {
    out << "from_point: ";
    rosidl_generator_traits::value_to_yaml(msg.from_point, out);
    out << ", ";
  }

  // member: to_point
  {
    out << "to_point: ";
    rosidl_generator_traits::value_to_yaml(msg.to_point, out);
    out << ", ";
  }

  // member: route_info
  {
    out << "route_info: ";
    to_flow_style_yaml(msg.route_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RouteConnection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: from_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "from_point: ";
    rosidl_generator_traits::value_to_yaml(msg.from_point, out);
    out << "\n";
  }

  // member: to_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "to_point: ";
    rosidl_generator_traits::value_to_yaml(msg.to_point, out);
    out << "\n";
  }

  // member: route_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "route_info:\n";
    to_block_style_yaml(msg.route_info, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RouteConnection & msg, bool use_flow_style = false)
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

}  // namespace topology_msgs

namespace rosidl_generator_traits
{

[[deprecated("use topology_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const topology_msgs::msg::RouteConnection & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::msg::RouteConnection & msg)
{
  return topology_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::msg::RouteConnection>()
{
  return "topology_msgs::msg::RouteConnection";
}

template<>
inline const char * name<topology_msgs::msg::RouteConnection>()
{
  return "topology_msgs/msg/RouteConnection";
}

template<>
struct has_fixed_size<topology_msgs::msg::RouteConnection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::msg::RouteConnection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::msg::RouteConnection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__TRAITS_HPP_
