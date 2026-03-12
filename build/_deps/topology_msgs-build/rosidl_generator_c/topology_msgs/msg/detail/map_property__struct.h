// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__STRUCT_H_
#define TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'support_controllers'
// Member 'support_goal_checkers'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MapProperty in the package topology_msgs.
typedef struct topology_msgs__msg__MapProperty
{
  rosidl_runtime_c__String__Sequence support_controllers;
  rosidl_runtime_c__String__Sequence support_goal_checkers;
} topology_msgs__msg__MapProperty;

// Struct for a sequence of topology_msgs__msg__MapProperty.
typedef struct topology_msgs__msg__MapProperty__Sequence
{
  topology_msgs__msg__MapProperty * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__msg__MapProperty__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__STRUCT_H_
