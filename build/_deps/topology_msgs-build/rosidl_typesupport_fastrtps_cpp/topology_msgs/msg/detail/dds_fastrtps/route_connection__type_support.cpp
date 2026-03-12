// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from topology_msgs:msg/RouteConnection.idl
// generated code does not contain a copyright notice
#include "topology_msgs/msg/detail/route_connection__rosidl_typesupport_fastrtps_cpp.hpp"
#include "topology_msgs/msg/detail/route_connection__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace topology_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const topology_msgs::msg::RouteInfo &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  topology_msgs::msg::RouteInfo &);
size_t get_serialized_size(
  const topology_msgs::msg::RouteInfo &,
  size_t current_alignment);
size_t
max_serialized_size_RouteInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace topology_msgs


namespace topology_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topology_msgs
cdr_serialize(
  const topology_msgs::msg::RouteConnection & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: from_point
  cdr << ros_message.from_point;
  // Member: to_point
  cdr << ros_message.to_point;
  // Member: route_info
  topology_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.route_info,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topology_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  topology_msgs::msg::RouteConnection & ros_message)
{
  // Member: from_point
  cdr >> ros_message.from_point;

  // Member: to_point
  cdr >> ros_message.to_point;

  // Member: route_info
  topology_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.route_info);

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topology_msgs
get_serialized_size(
  const topology_msgs::msg::RouteConnection & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: from_point
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.from_point.size() + 1);
  // Member: to_point
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.to_point.size() + 1);
  // Member: route_info

  current_alignment +=
    topology_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.route_info, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_topology_msgs
max_serialized_size_RouteConnection(
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


  // Member: from_point
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

  // Member: to_point
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

  // Member: route_info
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        topology_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_RouteInfo(
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
    using DataType = topology_msgs::msg::RouteConnection;
    is_plain =
      (
      offsetof(DataType, route_info) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RouteConnection__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const topology_msgs::msg::RouteConnection *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RouteConnection__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<topology_msgs::msg::RouteConnection *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RouteConnection__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const topology_msgs::msg::RouteConnection *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RouteConnection__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RouteConnection(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RouteConnection__callbacks = {
  "topology_msgs::msg",
  "RouteConnection",
  _RouteConnection__cdr_serialize,
  _RouteConnection__cdr_deserialize,
  _RouteConnection__get_serialized_size,
  _RouteConnection__max_serialized_size
};

static rosidl_message_type_support_t _RouteConnection__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RouteConnection__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace topology_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_topology_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<topology_msgs::msg::RouteConnection>()
{
  return &topology_msgs::msg::typesupport_fastrtps_cpp::_RouteConnection__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, topology_msgs, msg, RouteConnection)() {
  return &topology_msgs::msg::typesupport_fastrtps_cpp::_RouteConnection__handle;
}

#ifdef __cplusplus
}
#endif
