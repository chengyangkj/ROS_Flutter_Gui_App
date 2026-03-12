// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__STRUCT_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__topology_msgs__msg__MapProperty __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__msg__MapProperty __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapProperty_
{
  using Type = MapProperty_<ContainerAllocator>;

  explicit MapProperty_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MapProperty_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _support_controllers_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _support_controllers_type support_controllers;
  using _support_goal_checkers_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _support_goal_checkers_type support_goal_checkers;

  // setters for named parameter idiom
  Type & set__support_controllers(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->support_controllers = _arg;
    return *this;
  }
  Type & set__support_goal_checkers(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->support_goal_checkers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topology_msgs::msg::MapProperty_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::msg::MapProperty_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::MapProperty_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::MapProperty_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__msg__MapProperty
    std::shared_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__msg__MapProperty
    std::shared_ptr<topology_msgs::msg::MapProperty_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapProperty_ & other) const
  {
    if (this->support_controllers != other.support_controllers) {
      return false;
    }
    if (this->support_goal_checkers != other.support_goal_checkers) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapProperty_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapProperty_

// alias to use template instance with default allocator
using MapProperty =
  topology_msgs::msg::MapProperty_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__STRUCT_HPP_
