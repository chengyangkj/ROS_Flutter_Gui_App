// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__BUILDER_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topology_msgs/msg/detail/topology_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topology_msgs
{

namespace msg
{

namespace builder
{

class Init_TopologyMap_routes
{
public:
  explicit Init_TopologyMap_routes(::topology_msgs::msg::TopologyMap & msg)
  : msg_(msg)
  {}
  ::topology_msgs::msg::TopologyMap routes(::topology_msgs::msg::TopologyMap::_routes_type arg)
  {
    msg_.routes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMap msg_;
};

class Init_TopologyMap_points
{
public:
  explicit Init_TopologyMap_points(::topology_msgs::msg::TopologyMap & msg)
  : msg_(msg)
  {}
  Init_TopologyMap_routes points(::topology_msgs::msg::TopologyMap::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_TopologyMap_routes(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMap msg_;
};

class Init_TopologyMap_map_property
{
public:
  explicit Init_TopologyMap_map_property(::topology_msgs::msg::TopologyMap & msg)
  : msg_(msg)
  {}
  Init_TopologyMap_points map_property(::topology_msgs::msg::TopologyMap::_map_property_type arg)
  {
    msg_.map_property = std::move(arg);
    return Init_TopologyMap_points(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMap msg_;
};

class Init_TopologyMap_map_name
{
public:
  Init_TopologyMap_map_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TopologyMap_map_property map_name(::topology_msgs::msg::TopologyMap::_map_name_type arg)
  {
    msg_.map_name = std::move(arg);
    return Init_TopologyMap_map_property(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::msg::TopologyMap>()
{
  return topology_msgs::msg::builder::Init_TopologyMap_map_name();
}

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__BUILDER_HPP_
