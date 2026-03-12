// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "topology_msgs/msg/detail/map_property__struct.hpp"
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

void MapProperty_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) topology_msgs::msg::MapProperty(_init);
}

void MapProperty_fini_function(void * message_memory)
{
  auto typed_message = static_cast<topology_msgs::msg::MapProperty *>(message_memory);
  typed_message->~MapProperty();
}

size_t size_function__MapProperty__support_controllers(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapProperty__support_controllers(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MapProperty__support_controllers(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__MapProperty__support_controllers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__MapProperty__support_controllers(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__MapProperty__support_controllers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__MapProperty__support_controllers(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__MapProperty__support_controllers(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MapProperty__support_goal_checkers(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MapProperty__support_goal_checkers(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MapProperty__support_goal_checkers(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__MapProperty__support_goal_checkers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__MapProperty__support_goal_checkers(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__MapProperty__support_goal_checkers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__MapProperty__support_goal_checkers(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__MapProperty__support_goal_checkers(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MapProperty_message_member_array[2] = {
  {
    "support_controllers",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::MapProperty, support_controllers),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapProperty__support_controllers,  // size() function pointer
    get_const_function__MapProperty__support_controllers,  // get_const(index) function pointer
    get_function__MapProperty__support_controllers,  // get(index) function pointer
    fetch_function__MapProperty__support_controllers,  // fetch(index, &value) function pointer
    assign_function__MapProperty__support_controllers,  // assign(index, value) function pointer
    resize_function__MapProperty__support_controllers  // resize(index) function pointer
  },
  {
    "support_goal_checkers",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::MapProperty, support_goal_checkers),  // bytes offset in struct
    nullptr,  // default value
    size_function__MapProperty__support_goal_checkers,  // size() function pointer
    get_const_function__MapProperty__support_goal_checkers,  // get_const(index) function pointer
    get_function__MapProperty__support_goal_checkers,  // get(index) function pointer
    fetch_function__MapProperty__support_goal_checkers,  // fetch(index, &value) function pointer
    assign_function__MapProperty__support_goal_checkers,  // assign(index, value) function pointer
    resize_function__MapProperty__support_goal_checkers  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MapProperty_message_members = {
  "topology_msgs::msg",  // message namespace
  "MapProperty",  // message name
  2,  // number of fields
  sizeof(topology_msgs::msg::MapProperty),
  MapProperty_message_member_array,  // message members
  MapProperty_init_function,  // function to initialize message memory (memory has to be allocated)
  MapProperty_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MapProperty_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MapProperty_message_members,
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
get_message_type_support_handle<topology_msgs::msg::MapProperty>()
{
  return &::topology_msgs::msg::rosidl_typesupport_introspection_cpp::MapProperty_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, topology_msgs, msg, MapProperty)() {
  return &::topology_msgs::msg::rosidl_typesupport_introspection_cpp::MapProperty_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
