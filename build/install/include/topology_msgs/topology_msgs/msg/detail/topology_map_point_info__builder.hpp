// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topology_msgs:msg/TopologyMapPointInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__BUILDER_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topology_msgs/msg/detail/topology_map_point_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topology_msgs
{

namespace msg
{

namespace builder
{

class Init_TopologyMapPointInfo_type
{
public:
  explicit Init_TopologyMapPointInfo_type(::topology_msgs::msg::TopologyMapPointInfo & msg)
  : msg_(msg)
  {}
  ::topology_msgs::msg::TopologyMapPointInfo type(::topology_msgs::msg::TopologyMapPointInfo::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMapPointInfo msg_;
};

class Init_TopologyMapPointInfo_theta
{
public:
  explicit Init_TopologyMapPointInfo_theta(::topology_msgs::msg::TopologyMapPointInfo & msg)
  : msg_(msg)
  {}
  Init_TopologyMapPointInfo_type theta(::topology_msgs::msg::TopologyMapPointInfo::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_TopologyMapPointInfo_type(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMapPointInfo msg_;
};

class Init_TopologyMapPointInfo_y
{
public:
  explicit Init_TopologyMapPointInfo_y(::topology_msgs::msg::TopologyMapPointInfo & msg)
  : msg_(msg)
  {}
  Init_TopologyMapPointInfo_theta y(::topology_msgs::msg::TopologyMapPointInfo::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_TopologyMapPointInfo_theta(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMapPointInfo msg_;
};

class Init_TopologyMapPointInfo_x
{
public:
  explicit Init_TopologyMapPointInfo_x(::topology_msgs::msg::TopologyMapPointInfo & msg)
  : msg_(msg)
  {}
  Init_TopologyMapPointInfo_y x(::topology_msgs::msg::TopologyMapPointInfo::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_TopologyMapPointInfo_y(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMapPointInfo msg_;
};

class Init_TopologyMapPointInfo_name
{
public:
  Init_TopologyMapPointInfo_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TopologyMapPointInfo_x name(::topology_msgs::msg::TopologyMapPointInfo::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_TopologyMapPointInfo_x(msg_);
  }

private:
  ::topology_msgs::msg::TopologyMapPointInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::msg::TopologyMapPointInfo>()
{
  return topology_msgs::msg::builder::Init_TopologyMapPointInfo_name();
}

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__BUILDER_HPP_
