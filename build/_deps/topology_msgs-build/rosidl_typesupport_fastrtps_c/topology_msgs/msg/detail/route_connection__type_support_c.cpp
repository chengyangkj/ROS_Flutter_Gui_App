// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from topology_msgs:msg/RouteConnection.idl
// generated code does not contain a copyright notice
#include "topology_msgs/msg/detail/route_connection__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "topology_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "topology_msgs/msg/detail/route_connection__struct.h"
#include "topology_msgs/msg/detail/route_connection__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // from_point, to_point
#include "rosidl_runtime_c/string_functions.h"  // from_point, to_point
#include "topology_msgs/msg/detail/route_info__functions.h"  // route_info

// forward declare type support functions
size_t get_serialized_size_topology_msgs__msg__RouteInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_topology_msgs__msg__RouteInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topology_msgs, msg, RouteInfo)();


using _RouteConnection__ros_msg_type = topology_msgs__msg__RouteConnection;

static bool _RouteConnection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RouteConnection__ros_msg_type * ros_message = static_cast<const _RouteConnection__ros_msg_type *>(untyped_ros_message);
  // Field name: from_point
  {
    const rosidl_runtime_c__String * str = &ros_message->from_point;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: to_point
  {
    const rosidl_runtime_c__String * str = &ros_message->to_point;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: route_info
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, topology_msgs, msg, RouteInfo
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->route_info, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _RouteConnection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RouteConnection__ros_msg_type * ros_message = static_cast<_RouteConnection__ros_msg_type *>(untyped_ros_message);
  // Field name: from_point
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->from_point.data) {
      rosidl_runtime_c__String__init(&ros_message->from_point);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->from_point,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'from_point'\n");
      return false;
    }
  }

  // Field name: to_point
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->to_point.data) {
      rosidl_runtime_c__String__init(&ros_message->to_point);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->to_point,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'to_point'\n");
      return false;
    }
  }

  // Field name: route_info
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, topology_msgs, msg, RouteInfo
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->route_info))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_topology_msgs
size_t get_serialized_size_topology_msgs__msg__RouteConnection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RouteConnection__ros_msg_type * ros_message = static_cast<const _RouteConnection__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name from_point
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->from_point.size + 1);
  // field.name to_point
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->to_point.size + 1);
  // field.name route_info

  current_alignment += get_serialized_size_topology_msgs__msg__RouteInfo(
    &(ros_message->route_info), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _RouteConnection__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_topology_msgs__msg__RouteConnection(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_topology_msgs
size_t max_serialized_size_topology_msgs__msg__RouteConnection(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: from_point
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: to_point
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: route_info
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_topology_msgs__msg__RouteInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = topology_msgs__msg__RouteConnection;
    is_plain =
      (
      offsetof(DataType, route_info) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RouteConnection__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_topology_msgs__msg__RouteConnection(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RouteConnection = {
  "topology_msgs::msg",
  "RouteConnection",
  _RouteConnection__cdr_serialize,
  _RouteConnection__cdr_deserialize,
  _RouteConnection__get_serialized_size,
  _RouteConnection__max_serialized_size
};

static rosidl_message_type_support_t _RouteConnection__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RouteConnection,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, topology_msgs, msg, RouteConnection)() {
  return &_RouteConnection__type_support;
}

#if defined(__cplusplus)
}
#endif
