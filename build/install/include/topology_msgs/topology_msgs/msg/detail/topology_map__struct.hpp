// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__STRUCT_HPP_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'map_property'
#include "topology_msgs/msg/detail/map_property__struct.hpp"
// Member 'points'
#include "topology_msgs/msg/detail/topology_map_point_info__struct.hpp"
// Member 'routes'
#include "topology_msgs/msg/detail/route_connection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__topology_msgs__msg__TopologyMap __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__msg__TopologyMap __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TopologyMap_
{
  using Type = TopologyMap_<ContainerAllocator>;

  explicit TopologyMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_property(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_name = "";
    }
  }

  explicit TopologyMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_name(_alloc),
    map_property(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_name = "";
    }
  }

  // field types and members
  using _map_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_name_type map_name;
  using _map_property_type =
    topology_msgs::msg::MapProperty_<ContainerAllocator>;
  _map_property_type map_property;
  using _points_type =
    std::vector<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>>;
  _points_type points;
  using _routes_type =
    std::vector<topology_msgs::msg::RouteConnection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<topology_msgs::msg::RouteConnection_<ContainerAllocator>>>;
  _routes_type routes;

  // setters for named parameter idiom
  Type & set__map_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_name = _arg;
    return *this;
  }
  Type & set__map_property(
    const topology_msgs::msg::MapProperty_<ContainerAllocator> & _arg)
  {
    this->map_property = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<topology_msgs::msg::TopologyMapPointInfo_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__routes(
    const std::vector<topology_msgs::msg::RouteConnection_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<topology_msgs::msg::RouteConnection_<ContainerAllocator>>> & _arg)
  {
    this->routes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topology_msgs::msg::TopologyMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::msg::TopologyMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::TopologyMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::msg::TopologyMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__msg__TopologyMap
    std::shared_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__msg__TopologyMap
    std::shared_ptr<topology_msgs::msg::TopologyMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TopologyMap_ & other) const
  {
    if (this->map_name != other.map_name) {
      return false;
    }
    if (this->map_property != other.map_property) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->routes != other.routes) {
      return false;
    }
    return true;
  }
  bool operator!=(const TopologyMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TopologyMap_

// alias to use template instance with default allocator
using TopologyMap =
  topology_msgs::msg::TopologyMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__STRUCT_HPP_
