// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__BUILDER_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topology_msgs/msg/detail/map_property__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topology_msgs
{

namespace msg
{

namespace builder
{

class Init_MapProperty_support_goal_checkers
{
public:
  explicit Init_MapProperty_support_goal_checkers(::topology_msgs::msg::MapProperty & msg)
  : msg_(msg)
  {}
  ::topology_msgs::msg::MapProperty support_goal_checkers(::topology_msgs::msg::MapProperty::_support_goal_checkers_type arg)
  {
    msg_.support_goal_checkers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::msg::MapProperty msg_;
};

class Init_MapProperty_support_controllers
{
public:
  Init_MapProperty_support_controllers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapProperty_support_goal_checkers support_controllers(::topology_msgs::msg::MapProperty::_support_controllers_type arg)
  {
    msg_.support_controllers = std::move(arg);
    return Init_MapProperty_support_goal_checkers(msg_);
  }

private:
  ::topology_msgs::msg::MapProperty msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::msg::MapProperty>()
{
  return topology_msgs::msg::builder::Init_MapProperty_support_controllers();
}

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__BUILDER_HPP_
