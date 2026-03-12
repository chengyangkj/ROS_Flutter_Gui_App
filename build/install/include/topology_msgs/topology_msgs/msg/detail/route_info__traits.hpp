// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__TRAITS_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topology_msgs/msg/detail/route_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace topology_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RouteInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: controller
  {
    out << "controller: ";
    rosidl_generator_traits::value_to_yaml(msg.controller, out);
    out << ", ";
  }

  // member: goal_checker
  {
    out << "goal_checker: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_checker, out);
    out << ", ";
  }

  // member: speed_limit
  {
    out << "speed_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_limit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RouteInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: controller
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controller: ";
    rosidl_generator_traits::value_to_yaml(msg.controller, out);
    out << "\n";
  }

  // member: goal_checker
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_checker: ";
    rosidl_generator_traits::value_to_yaml(msg.goal_checker, out);
    out << "\n";
  }

  // member: speed_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_limit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RouteInfo & msg, bool use_flow_style = false)
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
  const topology_msgs::msg::RouteInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::msg::RouteInfo & msg)
{
  return topology_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::msg::RouteInfo>()
{
  return "topology_msgs::msg::RouteInfo";
}

template<>
inline const char * name<topology_msgs::msg::RouteInfo>()
{
  return "topology_msgs/msg/RouteInfo";
}

template<>
struct has_fixed_size<topology_msgs::msg::RouteInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::msg::RouteInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::msg::RouteInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__TRAITS_HPP_
