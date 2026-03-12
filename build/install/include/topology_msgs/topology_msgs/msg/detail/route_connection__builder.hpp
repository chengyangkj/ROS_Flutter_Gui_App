// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topology_msgs:msg/RouteConnection.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__BUILDER_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topology_msgs/msg/detail/route_connection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topology_msgs
{

namespace msg
{

namespace builder
{

class Init_RouteConnection_route_info
{
public:
  explicit Init_RouteConnection_route_info(::topology_msgs::msg::RouteConnection & msg)
  : msg_(msg)
  {}
  ::topology_msgs::msg::RouteConnection route_info(::topology_msgs::msg::RouteConnection::_route_info_type arg)
  {
    msg_.route_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::msg::RouteConnection msg_;
};

class Init_RouteConnection_to_point
{
public:
  explicit Init_RouteConnection_to_point(::topology_msgs::msg::RouteConnection & msg)
  : msg_(msg)
  {}
  Init_RouteConnection_route_info to_point(::topology_msgs::msg::RouteConnection::_to_point_type arg)
  {
    msg_.to_point = std::move(arg);
    return Init_RouteConnection_route_info(msg_);
  }

private:
  ::topology_msgs::msg::RouteConnection msg_;
};

class Init_RouteConnection_from_point
{
public:
  Init_RouteConnection_from_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RouteConnection_to_point from_point(::topology_msgs::msg::RouteConnection::_from_point_type arg)
  {
    msg_.from_point = std::move(arg);
    return Init_RouteConnection_to_point(msg_);
  }

private:
  ::topology_msgs::msg::RouteConnection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::msg::RouteConnection>()
{
  return topology_msgs::msg::builder::Init_RouteConnection_from_point();
}

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__BUILDER_HPP_
