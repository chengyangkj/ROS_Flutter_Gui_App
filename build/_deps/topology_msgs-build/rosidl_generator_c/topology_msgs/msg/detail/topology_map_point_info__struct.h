// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topology_msgs:msg/TopologyMapPointInfo.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__STRUCT_H_
#define TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NAV_GOAL'.
enum
{
  topology_msgs__msg__TopologyMapPointInfo__NAV_GOAL = 0
};

/// Constant 'CHARGE_STATION'.
enum
{
  topology_msgs__msg__TopologyMapPointInfo__CHARGE_STATION = 1
};

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TopologyMapPointInfo in the package topology_msgs.
typedef struct topology_msgs__msg__TopologyMapPointInfo
{
  rosidl_runtime_c__String name;
  double x;
  double y;
  double theta;
  uint8_t type;
} topology_msgs__msg__TopologyMapPointInfo;

// Struct for a sequence of topology_msgs__msg__TopologyMapPointInfo.
typedef struct topology_msgs__msg__TopologyMapPointInfo__Sequence
{
  topology_msgs__msg__TopologyMapPointInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__msg__TopologyMapPointInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__TOPOLOGY_MAP_POINT_INFO__STRUCT_H_
