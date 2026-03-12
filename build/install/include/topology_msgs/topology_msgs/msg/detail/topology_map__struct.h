// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__STRUCT_H_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'map_name'
#include "rosidl_runtime_c/string.h"
// Member 'map_property'
#include "topology_msgs/msg/detail/map_property__struct.h"
// Member 'points'
#include "topology_msgs/msg/detail/topology_map_point_info__struct.h"
// Member 'routes'
#include "topology_msgs/msg/detail/route_connection__struct.h"

/// Struct defined in msg/TopologyMap in the package topology_msgs.
typedef struct topology_msgs__msg__TopologyMap
{
  rosidl_runtime_c__String map_name;
  topology_msgs__msg__MapProperty map_property;
  topology_msgs__msg__TopologyMapPointInfo__Sequence points;
  topology_msgs__msg__RouteConnection__Sequence routes;
} topology_msgs__msg__TopologyMap;

// Struct for a sequence of topology_msgs__msg__TopologyMap.
typedef struct topology_msgs__msg__TopologyMap__Sequence
{
  topology_msgs__msg__TopologyMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__msg__TopologyMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP__STRUCT_H_
