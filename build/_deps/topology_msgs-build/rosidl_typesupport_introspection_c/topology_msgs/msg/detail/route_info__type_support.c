// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "topology_msgs/msg/detail/route_info__rosidl_typesupport_introspection_c.h"
#include "topology_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "topology_msgs/msg/detail/route_info__functions.h"
#include "topology_msgs/msg/detail/route_info__struct.h"


// Include directives for member types
// Member `controller`
// Member `goal_checker`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  topology_msgs__msg__RouteInfo__init(message_memory);
}

void topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_fini_function(void * message_memory)
{
  topology_msgs__msg__RouteInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_member_array[3] = {
  {
    "controller",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__RouteInfo, controller),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal_checker",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__RouteInfo, goal_checker),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__RouteInfo, speed_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_members = {
  "topology_msgs__msg",  // message namespace
  "RouteInfo",  // message name
  3,  // number of fields
  sizeof(topology_msgs__msg__RouteInfo),
  topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_member_array,  // message members
  topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_type_support_handle = {
  0,
  &topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_topology_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topology_msgs, msg, RouteInfo)() {
  if (!topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_type_support_handle.typesupport_identifier) {
    topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &topology_msgs__msg__RouteInfo__rosidl_typesupport_introspection_c__RouteInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
