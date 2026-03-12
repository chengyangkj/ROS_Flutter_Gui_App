// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "topology_msgs/msg/detail/map_property__rosidl_typesupport_introspection_c.h"
#include "topology_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "topology_msgs/msg/detail/map_property__functions.h"
#include "topology_msgs/msg/detail/map_property__struct.h"


// Include directives for member types
// Member `support_controllers`
// Member `support_goal_checkers`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  topology_msgs__msg__MapProperty__init(message_memory);
}

void topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_fini_function(void * message_memory)
{
  topology_msgs__msg__MapProperty__fini(message_memory);
}

size_t topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__size_function__MapProperty__support_controllers(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_const_function__MapProperty__support_controllers(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_function__MapProperty__support_controllers(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__fetch_function__MapProperty__support_controllers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_const_function__MapProperty__support_controllers(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__assign_function__MapProperty__support_controllers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_function__MapProperty__support_controllers(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__resize_function__MapProperty__support_controllers(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__size_function__MapProperty__support_goal_checkers(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_const_function__MapProperty__support_goal_checkers(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_function__MapProperty__support_goal_checkers(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__fetch_function__MapProperty__support_goal_checkers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_const_function__MapProperty__support_goal_checkers(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__assign_function__MapProperty__support_goal_checkers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_function__MapProperty__support_goal_checkers(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__resize_function__MapProperty__support_goal_checkers(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_member_array[2] = {
  {
    "support_controllers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__MapProperty, support_controllers),  // bytes offset in struct
    NULL,  // default value
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__size_function__MapProperty__support_controllers,  // size() function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_const_function__MapProperty__support_controllers,  // get_const(index) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_function__MapProperty__support_controllers,  // get(index) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__fetch_function__MapProperty__support_controllers,  // fetch(index, &value) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__assign_function__MapProperty__support_controllers,  // assign(index, value) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__resize_function__MapProperty__support_controllers  // resize(index) function pointer
  },
  {
    "support_goal_checkers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(topology_msgs__msg__MapProperty, support_goal_checkers),  // bytes offset in struct
    NULL,  // default value
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__size_function__MapProperty__support_goal_checkers,  // size() function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_const_function__MapProperty__support_goal_checkers,  // get_const(index) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__get_function__MapProperty__support_goal_checkers,  // get(index) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__fetch_function__MapProperty__support_goal_checkers,  // fetch(index, &value) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__assign_function__MapProperty__support_goal_checkers,  // assign(index, value) function pointer
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__resize_function__MapProperty__support_goal_checkers  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_members = {
  "topology_msgs__msg",  // message namespace
  "MapProperty",  // message name
  2,  // number of fields
  sizeof(topology_msgs__msg__MapProperty),
  topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_member_array,  // message members
  topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_init_function,  // function to initialize message memory (memory has to be allocated)
  topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_type_support_handle = {
  0,
  &topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_topology_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, topology_msgs, msg, MapProperty)() {
  if (!topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_type_support_handle.typesupport_identifier) {
    topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &topology_msgs__msg__MapProperty__rosidl_typesupport_introspection_c__MapProperty_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
