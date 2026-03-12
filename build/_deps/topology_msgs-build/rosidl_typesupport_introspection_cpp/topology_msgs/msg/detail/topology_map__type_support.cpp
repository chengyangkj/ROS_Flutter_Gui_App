// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "topology_msgs/msg/detail/topology_map__struct.hpp"
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

void TopologyMap_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) topology_msgs::msg::TopologyMap(_init);
}

void TopologyMap_fini_function(void * message_memory)
{
  auto typed_message = static_cast<topology_msgs::msg::TopologyMap *>(message_memory);
  typed_message->~TopologyMap();
}

size_t size_function__TopologyMap__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<topology_msgs::msg::TopologyMapPointInfo> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TopologyMap__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<topology_msgs::msg::TopologyMapPointInfo> *>(untyped_member);
  return &member[index];
}

void * get_function__TopologyMap__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<topology_msgs::msg::TopologyMapPointInfo> *>(untyped_member);
  return &member[index];
}

void fetch_function__TopologyMap__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const topology_msgs::msg::TopologyMapPointInfo *>(
    get_const_function__TopologyMap__points(untyped_member, index));
  auto & value = *reinterpret_cast<topology_msgs::msg::TopologyMapPointInfo *>(untyped_value);
  value = item;
}

void assign_function__TopologyMap__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<topology_msgs::msg::TopologyMapPointInfo *>(
    get_function__TopologyMap__points(untyped_member, index));
  const auto & value = *reinterpret_cast<const topology_msgs::msg::TopologyMapPointInfo *>(untyped_value);
  item = value;
}

void resize_function__TopologyMap__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<topology_msgs::msg::TopologyMapPointInfo> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TopologyMap__routes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<topology_msgs::msg::RouteConnection> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TopologyMap__routes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<topology_msgs::msg::RouteConnection> *>(untyped_member);
  return &member[index];
}

void * get_function__TopologyMap__routes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<topology_msgs::msg::RouteConnection> *>(untyped_member);
  return &member[index];
}

void fetch_function__TopologyMap__routes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const topology_msgs::msg::RouteConnection *>(
    get_const_function__TopologyMap__routes(untyped_member, index));
  auto & value = *reinterpret_cast<topology_msgs::msg::RouteConnection *>(untyped_value);
  value = item;
}

void assign_function__TopologyMap__routes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<topology_msgs::msg::RouteConnection *>(
    get_function__TopologyMap__routes(untyped_member, index));
  const auto & value = *reinterpret_cast<const topology_msgs::msg::RouteConnection *>(untyped_value);
  item = value;
}

void resize_function__TopologyMap__routes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<topology_msgs::msg::RouteConnection> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TopologyMap_message_member_array[4] = {
  {
    "map_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::TopologyMap, map_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "map_property",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<topology_msgs::msg::MapProperty>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::TopologyMap, map_property),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<topology_msgs::msg::TopologyMapPointInfo>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::TopologyMap, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__TopologyMap__points,  // size() function pointer
    get_const_function__TopologyMap__points,  // get_const(index) function pointer
    get_function__TopologyMap__points,  // get(index) function pointer
    fetch_function__TopologyMap__points,  // fetch(index, &value) function pointer
    assign_function__TopologyMap__points,  // assign(index, value) function pointer
    resize_function__TopologyMap__points  // resize(index) function pointer
  },
  {
    "routes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<topology_msgs::msg::RouteConnection>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs::msg::TopologyMap, routes),  // bytes offset in struct
    nullptr,  // default value
    size_function__TopologyMap__routes,  // size() function pointer
    get_const_function__TopologyMap__routes,  // get_const(index) function pointer
    get_function__TopologyMap__routes,  // get(index) function pointer
    fetch_function__TopologyMap__routes,  // fetch(index, &value) function pointer
    assign_function__TopologyMap__routes,  // assign(index, value) function pointer
    resize_function__TopologyMap__routes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TopologyMap_message_members = {
  "topology_msgs::msg",  // message namespace
  "TopologyMap",  // message name
  4,  // number of fields
  sizeof(topology_msgs::msg::TopologyMap),
  TopologyMap_message_member_array,  // message members
  TopologyMap_init_function,  // function to initialize message memory (memory has to be allocated)
  TopologyMap_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TopologyMap_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TopologyMap_message_members,
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
get_message_type_support_handle<topology_msgs::msg::TopologyMap>()
{
  return &::topology_msgs::msg::rosidl_typesupport_introspection_cpp::TopologyMap_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, topology_msgs, msg, TopologyMap)() {
  return &::topology_msgs::msg::rosidl_typesupport_introspection_cpp::TopologyMap_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
