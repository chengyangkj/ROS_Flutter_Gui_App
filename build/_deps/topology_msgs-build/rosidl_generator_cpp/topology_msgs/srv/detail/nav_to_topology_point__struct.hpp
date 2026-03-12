// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from topology_msgs:srv/NavToTopologyPoint.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__STRUCT_HPP_
#define TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Request __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NavToTopologyPoint_Request_
{
  using Type = NavToTopologyPoint_Request_<ContainerAllocator>;

  explicit NavToTopologyPoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->point_name = "";
    }
  }

  explicit NavToTopologyPoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->point_name = "";
    }
  }

  // field types and members
  using _point_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _point_name_type point_name;

  // setters for named parameter idiom
  Type & set__point_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->point_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Request
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Request
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavToTopologyPoint_Request_ & other) const
  {
    if (this->point_name != other.point_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavToTopologyPoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavToTopologyPoint_Request_

// alias to use template instance with default allocator
using NavToTopologyPoint_Request =
  topology_msgs::srv::NavToTopologyPoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace topology_msgs


#ifndef _WIN32
# define DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Response __declspec(deprecated)
#endif

namespace topology_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NavToTopologyPoint_Response_
{
  using Type = NavToTopologyPoint_Response_<ContainerAllocator>;

  explicit NavToTopologyPoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_success = false;
      this->task_id = "";
      this->msg = "";
    }
  }

  explicit NavToTopologyPoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_id(_alloc),
    msg(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_success = false;
      this->task_id = "";
      this->msg = "";
    }
  }

  // field types and members
  using _is_success_type =
    bool;
  _is_success_type is_success;
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _msg_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _msg_type msg;

  // setters for named parameter idiom
  Type & set__is_success(
    const bool & _arg)
  {
    this->is_success = _arg;
    return *this;
  }
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__msg(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->msg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Response
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__topology_msgs__srv__NavToTopologyPoint_Response
    std::shared_ptr<topology_msgs::srv::NavToTopologyPoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavToTopologyPoint_Response_ & other) const
  {
    if (this->is_success != other.is_success) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->msg != other.msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavToTopologyPoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavToTopologyPoint_Response_

// alias to use template instance with default allocator
using NavToTopologyPoint_Response =
  topology_msgs::srv::NavToTopologyPoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace topology_msgs

namespace topology_msgs
{

namespace srv
{

struct NavToTopologyPoint
{
  using Request = topology_msgs::srv::NavToTopologyPoint_Request;
  using Response = topology_msgs::srv::NavToTopologyPoint_Response;
};

}  // namespace srv

}  // namespace topology_msgs

#endif  // TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__STRUCT_HPP_
