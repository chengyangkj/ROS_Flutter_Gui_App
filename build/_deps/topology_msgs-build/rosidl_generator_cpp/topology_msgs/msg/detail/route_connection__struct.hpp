// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topology_msgs:msg/RouteConnection.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__STRUCT_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'route_info'
#include "topology_msgs/msg/detail/route_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__topology_msgs__msg__RouteConnection __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__msg__RouteConnection __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RouteConnection_
{
  using Type = RouteConnection_<ContainerAllocator>;

  explicit RouteConnection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : route_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->from_point = "";
      this->to_point = "";
    }
  }

  explicit RouteConnection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : from_point(_alloc),
    to_point(_alloc),
    route_info(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->from_point = "";
      this->to_point = "";
    }
  }

  // field types and members
  using _from_point_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _from_point_type from_point;
  using _to_point_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _to_point_type to_point;
  using _route_info_type =
    topology_msgs::msg::RouteInfo_<ContainerAllocator>;
  _route_info_type route_info;

  // setters for named parameter idiom
  Type & set__from_point(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->from_point = _arg;
    return *this;
  }
  Type & set__to_point(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->to_point = _arg;
    return *this;
  }
  Type & set__route_info(
    const topology_msgs::msg::RouteInfo_<ContainerAllocator> & _arg)
  {
    this->route_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topology_msgs::msg::RouteConnection_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::msg::RouteConnection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::RouteConnection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::RouteConnection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__msg__RouteConnection
    std::shared_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__msg__RouteConnection
    std::shared_ptr<topology_msgs::msg::RouteConnection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RouteConnection_ & other) const
  {
    if (this->from_point != other.from_point) {
      return false;
    }
    if (this->to_point != other.to_point) {
      return false;
    }
    if (this->route_info != other.route_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const RouteConnection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RouteConnection_

// alias to use template instance with default allocator
using RouteConnection =
  topology_msgs::msg::RouteConnection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__STRUCT_HPP_
