// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topology_msgs:msg/TopologyMapPointInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__STRUCT_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__topology_msgs__msg__TopologyMapPointInfo __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__msg__TopologyMapPointInfo __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TopologyMapPointInfo_
{
  using Type = TopologyMapPointInfo_<ContainerAllocator>;

  explicit TopologyMapPointInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
      this->type = 0;
    }
  }

  explicit TopologyMapPointInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
      this->type = 0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _theta_type =
    double;
  _theta_type theta;
  using _type_type =
    uint8_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NAV_GOAL =
    0u;
  static constexpr uint8_t CHARGE_STATION =
    1u;

  // pointer types
  using RawPtr =
    topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__msg__TopologyMapPointInfo
    std::shared_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__msg__TopologyMapPointInfo
    std::shared_ptr<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TopologyMapPointInfo_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const TopologyMapPointInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TopologyMapPointInfo_

// alias to use template instance with default allocator
using TopologyMapPointInfo =
  topology_msgs::msg::TopologyMapPointInfo_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TopologyMapPointInfo_<ContainerAllocator>::NAV_GOAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TopologyMapPointInfo_<ContainerAllocator>::CHARGE_STATION;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__STRUCT_HPP_
