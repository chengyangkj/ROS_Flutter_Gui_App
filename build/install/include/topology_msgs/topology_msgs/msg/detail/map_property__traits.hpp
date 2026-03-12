// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__TRAITS_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topology_msgs/msg/detail/map_property__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace topology_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MapProperty & msg,
  std::ostream & out)
{
  out << "{";
  // member: support_controllers
  {
    if (msg.support_controllers.size() == 0) {
      out << "support_controllers: []";
    } else {
      out << "support_controllers: [";
      size_t pending_items = msg.support_controllers.size();
      for (auto item : msg.support_controllers) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: support_goal_checkers
  {
    if (msg.support_goal_checkers.size() == 0) {
      out << "support_goal_checkers: []";
    } else {
      out << "support_goal_checkers: [";
      size_t pending_items = msg.support_goal_checkers.size();
      for (auto item : msg.support_goal_checkers) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const MapProperty & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: support_controllers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.support_controllers.size() == 0) {
      out << "support_controllers: []\n";
    } else {
      out << "support_controllers:\n";
      for (auto item : msg.support_controllers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: support_goal_checkers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.support_goal_checkers.size() == 0) {
      out << "support_goal_checkers: []\n";
    } else {
      out << "support_goal_checkers:\n";
      for (auto item : msg.support_goal_checkers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MapProperty & msg, bool use_flow_style = false)
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
  const topology_msgs::msg::MapProperty & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::msg::MapProperty & msg)
{
  return topology_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::msg::MapProperty>()
{
  return "topology_msgs::msg::MapProperty";
}

template<>
inline const char * name<topology_msgs::msg::MapProperty>()
{
  return "topology_msgs/msg/MapProperty";
}

template<>
struct has_fixed_size<topology_msgs::msg::MapProperty>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::msg::MapProperty>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::msg::MapProperty>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__TRAITS_HPP_
