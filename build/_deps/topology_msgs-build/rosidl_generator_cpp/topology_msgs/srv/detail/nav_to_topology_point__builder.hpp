// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from topology_msgs:srv/NavToTopologyPoint.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__BUILDER_HPP_
#define TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "topology_msgs/srv/detail/nav_to_topology_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace topology_msgs
{

namespace srv
{

namespace builder
{

class Init_NavToTopologyPoint_Request_point_name
{
public:
  Init_NavToTopologyPoint_Request_point_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::topology_msgs::srv::NavToTopologyPoint_Request point_name(::topology_msgs::srv::NavToTopologyPoint_Request::_point_name_type arg)
  {
    msg_.point_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::srv::NavToTopologyPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::srv::NavToTopologyPoint_Request>()
{
  return topology_msgs::srv::builder::Init_NavToTopologyPoint_Request_point_name();
}

}  // namespace topology_msgs


namespace topology_msgs
{

namespace srv
{

namespace builder
{

class Init_NavToTopologyPoint_Response_msg
{
public:
  explicit Init_NavToTopologyPoint_Response_msg(::topology_msgs::srv::NavToTopologyPoint_Response & msg)
  : msg_(msg)
  {}
  ::topology_msgs::srv::NavToTopologyPoint_Response msg(::topology_msgs::srv::NavToTopologyPoint_Response::_msg_type arg)
  {
    msg_.msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::topology_msgs::srv::NavToTopologyPoint_Response msg_;
};

class Init_NavToTopologyPoint_Response_task_id
{
public:
  explicit Init_NavToTopologyPoint_Response_task_id(::topology_msgs::srv::NavToTopologyPoint_Response & msg)
  : msg_(msg)
  {}
  Init_NavToTopologyPoint_Response_msg task_id(::topology_msgs::srv::NavToTopologyPoint_Response::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_NavToTopologyPoint_Response_msg(msg_);
  }

private:
  ::topology_msgs::srv::NavToTopologyPoint_Response msg_;
};

class Init_NavToTopologyPoint_Response_is_success
{
public:
  Init_NavToTopologyPoint_Response_is_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavToTopologyPoint_Response_task_id is_success(::topology_msgs::srv::NavToTopologyPoint_Response::_is_success_type arg)
  {
    msg_.is_success = std::move(arg);
    return Init_NavToTopologyPoint_Response_task_id(msg_);
  }

private:
  ::topology_msgs::srv::NavToTopologyPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::topology_msgs::srv::NavToTopologyPoint_Response>()
{
  return topology_msgs::srv::builder::Init_NavToTopologyPoint_Response_is_success();
}

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__BUILDER_HPP_
