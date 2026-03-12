// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topology_msgs:msg/RouteConnection.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__STRUCT_H_
#define TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'from_point'
// Member 'to_point'
#include "rosidl_runtime_c/string.h"
// Member 'route_info'
#include "topology_msgs/msg/detail/route_info__struct.h"

/// Struct defined in msg/RouteConnection in the package topology_msgs.
typedef struct topology_msgs__msg__RouteConnection
{
  rosidl_runtime_c__String from_point;
  rosidl_runtime_c__String to_point;
  topology_msgs__msg__RouteInfo route_info;
} topology_msgs__msg__RouteConnection;

// Struct for a sequence of topology_msgs__msg__RouteConnection.
typedef struct topology_msgs__msg__RouteConnection__Sequence
{
  topology_msgs__msg__RouteConnection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__msg__RouteConnection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__ROUTE_CONNECTION__STRUCT_H_
