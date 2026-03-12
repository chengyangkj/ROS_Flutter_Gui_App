// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from topology_msgs:msg/MapProperty.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__FUNCTIONS_H_
#define TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "topology_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "topology_msgs/msg/detail/map_property__struct.h"

/// Initialize msg/MapProperty message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * topology_msgs__msg__MapProperty
 * )) before or use
 * topology_msgs__msg__MapProperty__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__msg__MapProperty__init(topology_msgs__msg__MapProperty * msg);

/// Finalize msg/MapProperty message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__msg__MapProperty__fini(topology_msgs__msg__MapProperty * msg);

/// Create msg/MapProperty message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * topology_msgs__msg__MapProperty__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
topology_msgs__msg__MapProperty *
topology_msgs__msg__MapProperty__create();

/// Destroy msg/MapProperty message.
/**
 * It calls
 * topology_msgs__msg__MapProperty__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__msg__MapProperty__destroy(topology_msgs__msg__MapProperty * msg);

/// Check for msg/MapProperty message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__msg__MapProperty__are_equal(const topology_msgs__msg__MapProperty * lhs, const topology_msgs__msg__MapProperty * rhs);

/// Copy a msg/MapProperty message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__msg__MapProperty__copy(
  const topology_msgs__msg__MapProperty * input,
  topology_msgs__msg__MapProperty * output);

/// Initialize array of msg/MapProperty messages.
/**
 * It allocates the memory for the number of elements and calls
 * topology_msgs__msg__MapProperty__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__msg__MapProperty__Sequence__init(topology_msgs__msg__MapProperty__Sequence * array, size_t size);

/// Finalize array of msg/MapProperty messages.
/**
 * It calls
 * topology_msgs__msg__MapProperty__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__msg__MapProperty__Sequence__fini(topology_msgs__msg__MapProperty__Sequence * array);

/// Create array of msg/MapProperty messages.
/**
 * It allocates the memory for the array and calls
 * topology_msgs__msg__MapProperty__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
topology_msgs__msg__MapProperty__Sequence *
topology_msgs__msg__MapProperty__Sequence__create(size_t size);

/// Destroy array of msg/MapProperty messages.
/**
 * It calls
 * topology_msgs__msg__MapProperty__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__msg__MapProperty__Sequence__destroy(topology_msgs__msg__MapProperty__Sequence * array);

/// Check for msg/MapProperty message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__msg__MapProperty__Sequence__are_equal(const topology_msgs__msg__MapProperty__Sequence * lhs, const topology_msgs__msg__MapProperty__Sequence * rhs);

/// Copy an array of msg/MapProperty messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__msg__MapProperty__Sequence__copy(
  const topology_msgs__msg__MapProperty__Sequence * input,
  topology_msgs__msg__MapProperty__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__MSG__DETAIL__MAP_PROPERTY__FUNCTIONS_H_
