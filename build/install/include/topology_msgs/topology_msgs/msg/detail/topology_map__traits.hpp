// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__TRAITS_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "topology_msgs/msg/detail/topology_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'map_property'
#include "topology_msgs/msg/detail/map_property__traits.hpp"
// Member 'points'
#include "topology_msgs/msg/detail/topology_map_point_info__traits.hpp"
// Member 'routes'
#include "topology_msgs/msg/detail/route_connection__traits.hpp"

namespace topology_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TopologyMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: map_name
  {
    out << "map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.map_name, out);
    out << ", ";
  }

  // member: map_property
  {
    out << "map_property: ";
    to_flow_style_yaml(msg.map_property, out);
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: routes
  {
    if (msg.routes.size() == 0) {
      out << "routes: []";
    } else {
      out << "routes: [";
      size_t pending_items = msg.routes.size();
      for (auto item : msg.routes) {
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
  const TopologyMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_name: ";
    rosidl_generator_traits::value_to_yaml(msg.map_name, out);
    out << "\n";
  }

  // member: map_property
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_property:\n";
    to_block_style_yaml(msg.map_property, out, indentation + 2);
  }

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: routes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.routes.size() == 0) {
      out << "routes: []\n";
    } else {
      out << "routes:\n";
      for (auto item : msg.routes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TopologyMap & msg, bool use_flow_style = false)
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
  const topology_msgs::msg::TopologyMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  topology_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use topology_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const topology_msgs::msg::TopologyMap & msg)
{
  return topology_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<topology_msgs::msg::TopologyMap>()
{
  return "topology_msgs::msg::TopologyMap";
}

template<>
inline const char * name<topology_msgs::msg::TopologyMap>()
{
  return "topology_msgs/msg/TopologyMap";
}

template<>
struct has_fixed_size<topology_msgs::msg::TopologyMap>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<topology_msgs::msg::TopologyMap>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<topology_msgs::msg::TopologyMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__TRAITS_HPP_
