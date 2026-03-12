// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__BUILDER_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topology_msgs/msg/detail/route_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topology_msgs
{

namespace msg
{

namespace builder
{

class Init_RouteInfo_speed_limit
{
public:
  explicit Init_RouteInfo_speed_limit(::topology_msgs::msg::RouteInfo & msg)
  : msg_(msg)
  {}
  ::topology_msgs::msg::RouteInfo speed_limit(::topology_msgs::msg::RouteInfo::_speed_limit_type arg)
  {
    msg_.speed_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::msg::RouteInfo msg_;
};

class Init_RouteInfo_goal_checker
{
public:
  explicit Init_RouteInfo_goal_checker(::topology_msgs::msg::RouteInfo & msg)
  : msg_(msg)
  {}
  Init_RouteInfo_speed_limit goal_checker(::topology_msgs::msg::RouteInfo::_goal_checker_type arg)
  {
    msg_.goal_checker = std::move(arg);
    return Init_RouteInfo_speed_limit(msg_);
  }

private:
  ::topology_msgs::msg::RouteInfo msg_;
};

class Init_RouteInfo_controller
{
public:
  Init_RouteInfo_controller()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RouteInfo_goal_checker controller(::topology_msgs::msg::RouteInfo::_controller_type arg)
  {
    msg_.controller = std::move(arg);
    return Init_RouteInfo_goal_checker(msg_);
  }

private:
  ::topology_msgs::msg::RouteInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::msg::RouteInfo>()
{
  return topology_msgs::msg::builder::Init_RouteInfo_controller();
}

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__BUILDER_HPP_
