// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from topology_msgs:srv/NavToTopologyPoint.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__STRUCT_H_
#define TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'point_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/NavToTopologyPoint in the package topology_msgs.
typedef struct topology_msgs__srv__NavToTopologyPoint_Request
{
  rosidl_runtime_c__String point_name;
} topology_msgs__srv__NavToTopologyPoint_Request;

// Struct for a sequence of topology_msgs__srv__NavToTopologyPoint_Request.
typedef struct topology_msgs__srv__NavToTopologyPoint_Request__Sequence
{
  topology_msgs__srv__NavToTopologyPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__srv__NavToTopologyPoint_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'task_id'
// Member 'msg'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/NavToTopologyPoint in the package topology_msgs.
typedef struct topology_msgs__srv__NavToTopologyPoint_Response
{
  bool is_success;
  rosidl_runtime_c__String task_id;
  rosidl_runtime_c__String msg;
} topology_msgs__srv__NavToTopologyPoint_Response;

// Struct for a sequence of topology_msgs__srv__NavToTopologyPoint_Response.
typedef struct topology_msgs__srv__NavToTopologyPoint_Response__Sequence
{
  topology_msgs__srv__NavToTopologyPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} topology_msgs__srv__NavToTopologyPoint_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__STRUCT_H_
