// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__STRUCT_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__topology_msgs__msg__RouteInfo __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__msg__RouteInfo __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RouteInfo_
{
  using Type = RouteInfo_<ContainerAllocator>;

  explicit RouteInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller = "";
      this->goal_checker = "";
      this->speed_limit = 0.0f;
    }
  }

  explicit RouteInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : controller(_alloc),
    goal_checker(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller = "";
      this->goal_checker = "";
      this->speed_limit = 0.0f;
    }
  }

  // field types and members
  using _controller_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _controller_type controller;
  using _goal_checker_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _goal_checker_type goal_checker;
  using _speed_limit_type =
    float;
  _speed_limit_type speed_limit;

  // setters for named parameter idiom
  Type & set__controller(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->controller = _arg;
    return *this;
  }
  Type & set__goal_checker(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->goal_checker = _arg;
    return *this;
  }
  Type & set__speed_limit(
    const float & _arg)
  {
    this->speed_limit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topology_msgs::msg::RouteInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::msg::RouteInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::RouteInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::RouteInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__msg__RouteInfo
    std::shared_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__msg__RouteInfo
    std::shared_ptr<topology_msgs::msg::RouteInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RouteInfo_ & other) const
  {
    if (this->controller != other.controller) {
      return false;
    }
    if (this->goal_checker != other.goal_checker) {
      return false;
    }
    if (this->speed_limit != other.speed_limit) {
      return false;
    }
    return true;
  }
  bool operator!=(const RouteInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RouteInfo_

// alias to use template instance with default allocator
using RouteInfo =
  topology_msgs::msg::RouteInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__STRUCT_HPP_
