// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topology_msgs:msg/TopologyMapPointInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__TRAITS_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topology_msgs/msg/detail/topology_map_point_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace topology_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TopologyMapPointInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
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
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TopologyMapPointInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
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

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TopologyMapPointInfo & msg, bool use_flow_style = false)
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
  const topology_msgs::msg::TopologyMapPointInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::msg::TopologyMapPointInfo & msg)
{
  return topology_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::msg::TopologyMapPointInfo>()
{
  return "topology_msgs::msg::TopologyMapPointInfo";
}

template<>
inline const char * name<topology_msgs::msg::TopologyMapPointInfo>()
{
  return "topology_msgs/msg/TopologyMapPointInfo";
}

template<>
struct has_fixed_size<topology_msgs::msg::TopologyMapPointInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::msg::TopologyMapPointInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::msg::TopologyMapPointInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__TRAITS_HPP_
