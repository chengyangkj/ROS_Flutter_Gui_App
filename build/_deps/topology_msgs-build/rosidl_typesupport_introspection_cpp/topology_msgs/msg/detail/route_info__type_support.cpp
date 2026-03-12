// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "topology_msgs/msg/detail/route_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace topology_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RouteInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) topology_msgs::msg::RouteInfo(_init);
}

void RouteInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<topology_msgs::msg::RouteInfo *>(message_memory);
  typed_message->~RouteInfo();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RouteInfo_message_member_array[3] = {
  {
    "controller",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::RouteInfo, controller),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal_checker",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::RouteInfo, goal_checker),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed_limit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::RouteInfo, speed_limit),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RouteInfo_message_members = {
  "topology_msgs::msg",  // message namespace
  "RouteInfo",  // message name
  3,  // number of fields
  sizeof(topology_msgs::msg::RouteInfo),
  RouteInfo_message_member_array,  // message members
  RouteInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  RouteInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RouteInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RouteInfo_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace topology_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<topology_msgs::msg::RouteInfo>()
{
  return &::topology_msgs::msg::rosidl_typesupport_introspection_cpp::RouteInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, topology_msgs, msg, RouteInfo)() {
  return &::topology_msgs::msg::rosidl_typesupport_introspection_cpp::RouteInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
