// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__STRUCT_H_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'controller'
// Member 'goal_checker'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RouteInfo in the package topology_msgs.
typedef struct topology_msgs__msg__RouteInfo
{
  rosidl_runtime_c__String controller;
  rosidl_runtime_c__String goal_checker;
  float speed_limit;
} topology_msgs__msg__RouteInfo;

// Struct for a sequence of topology_msgs__msg__RouteInfo.
typedef struct topology_msgs__msg__RouteInfo__Sequence
{
  topology_msgs__msg__RouteInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__msg__RouteInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_INFO__STRUCT_H_
