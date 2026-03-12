// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "topology_msgs/msg/detail/topology_map__rosidl_typesupport_introspection_c.h"
#include "topology_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "topology_msgs/msg/detail/topology_map__functions.h"
#include "topology_msgs/msg/detail/topology_map__struct.h"


// Include directives for member types
// Member `map_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `map_property`
#include "topology_msgs/msg/map_property.h"
// Member `map_property`
#include "topology_msgs/msg/detail/map_property__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "topology_msgs/msg/topology_map_point_info.h"
// Member `points`
#include "topology_msgs/msg/detail/topology_map_point_info__rosidl_typesupport_introspection_c.h"
// Member `routes`
#include "topology_msgs/msg/route_connection.h"
// Member `routes`
#include "topology_msgs/msg/detail/route_connection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  topology_msgs__msg__TopologyMap__init(message_memory);
}

void topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_fini_function(void * message_memory)
{
  topology_msgs__msg__TopologyMap__fini(message_memory);
}

size_t topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__size_function__TopologyMap__points(
  const void * untyped_member)
{
  const topology_msgs__msg__TopologyMapPointInfo__Sequence * member =
    (const topology_msgs__msg__TopologyMapPointInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_const_function__TopologyMap__points(
  const void * untyped_member, size_t index)
{
  const topology_msgs__msg__TopologyMapPointInfo__Sequence * member =
    (const topology_msgs__msg__TopologyMapPointInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_function__TopologyMap__points(
  void * untyped_member, size_t index)
{
  topology_msgs__msg__TopologyMapPointInfo__Sequence * member =
    (topology_msgs__msg__TopologyMapPointInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__fetch_function__TopologyMap__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const topology_msgs__msg__TopologyMapPointInfo * item =
    ((const topology_msgs__msg__TopologyMapPointInfo *)
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_const_function__TopologyMap__points(untyped_member, index));
  topology_msgs__msg__TopologyMapPointInfo * value =
    (topology_msgs__msg__TopologyMapPointInfo *)(untyped_value);
  *value = *item;
}

void topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__assign_function__TopologyMap__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  topology_msgs__msg__TopologyMapPointInfo * item =
    ((topology_msgs__msg__TopologyMapPointInfo *)
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_function__TopologyMap__points(untyped_member, index));
  const topology_msgs__msg__TopologyMapPointInfo * value =
    (const topology_msgs__msg__TopologyMapPointInfo *)(untyped_value);
  *item = *value;
}

bool topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__resize_function__TopologyMap__points(
  void * untyped_member, size_t size)
{
  topology_msgs__msg__TopologyMapPointInfo__Sequence * member =
    (topology_msgs__msg__TopologyMapPointInfo__Sequence *)(untyped_member);
  topology_msgs__msg__TopologyMapPointInfo__Sequence__fini(member);
  return topology_msgs__msg__TopologyMapPointInfo__Sequence__init(member, size);
}

size_t topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__size_function__TopologyMap__routes(
  const void * untyped_member)
{
  const topology_msgs__msg__RouteConnection__Sequence * member =
    (const topology_msgs__msg__RouteConnection__Sequence *)(untyped_member);
  return member->size;
}

const void * topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_const_function__TopologyMap__routes(
  const void * untyped_member, size_t index)
{
  const topology_msgs__msg__RouteConnection__Sequence * member =
    (const topology_msgs__msg__RouteConnection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_function__TopologyMap__routes(
  void * untyped_member, size_t index)
{
  topology_msgs__msg__RouteConnection__Sequence * member =
    (topology_msgs__msg__RouteConnection__Sequence *)(untyped_member);
  return &member->data[index];
}

void topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__fetch_function__TopologyMap__routes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const topology_msgs__msg__RouteConnection * item =
    ((const topology_msgs__msg__RouteConnection *)
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_const_function__TopologyMap__routes(untyped_member, index));
  topology_msgs__msg__RouteConnection * value =
    (topology_msgs__msg__RouteConnection *)(untyped_value);
  *value = *item;
}

void topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__assign_function__TopologyMap__routes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  topology_msgs__msg__RouteConnection * item =
    ((topology_msgs__msg__RouteConnection *)
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_function__TopologyMap__routes(untyped_member, index));
  const topology_msgs__msg__RouteConnection * value =
    (const topology_msgs__msg__RouteConnection *)(untyped_value);
  *item = *value;
}

bool topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__resize_function__TopologyMap__routes(
  void * untyped_member, size_t size)
{
  topology_msgs__msg__RouteConnection__Sequence * member =
    (topology_msgs__msg__RouteConnection__Sequence *)(untyped_member);
  topology_msgs__msg__RouteConnection__Sequence__fini(member);
  return topology_msgs__msg__RouteConnection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_member_array[4] = {
  {
    "map_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__TopologyMap, map_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_property",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__TopologyMap, map_property),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__TopologyMap, points),  // bytes offset in struct
    NULL,  // default value
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__size_function__TopologyMap__points,  // size() function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_const_function__TopologyMap__points,  // get_const(index) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_function__TopologyMap__points,  // get(index) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__fetch_function__TopologyMap__points,  // fetch(index, &value) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__assign_function__TopologyMap__points,  // assign(index, value) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__resize_function__TopologyMap__points  // resize(index) function pointer
  },
  {
    "routes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__TopologyMap, routes),  // bytes offset in struct
    NULL,  // default value
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__size_function__TopologyMap__routes,  // size() function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_const_function__TopologyMap__routes,  // get_const(index) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__get_function__TopologyMap__routes,  // get(index) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__fetch_function__TopologyMap__routes,  // fetch(index, &value) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__assign_function__TopologyMap__routes,  // assign(index, value) function pointer
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__resize_function__TopologyMap__routes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_members = {
  "topology_msgs__msg",  // message namespace
  "TopologyMap",  // message name
  4,  // number of fields
  sizeof(topology_msgs__msg__TopologyMap),
  topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_member_array,  // message members
  topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_init_function,  // function to initialize message memory (memory has to be allocated)
  topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_type_support_handle = {
  0,
  &topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_topology_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topology_msgs, msg, TopologyMap)() {
  topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topology_msgs, msg, MapProperty)();
  topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topology_msgs, msg, TopologyMapPointInfo)();
  topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topology_msgs, msg, RouteConnection)();
  if (!topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_type_support_handle.typesupport_identifier) {
    topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &topology_msgs__msg__TopologyMap__rosidl_typesupport_introspection_c__TopologyMap_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
