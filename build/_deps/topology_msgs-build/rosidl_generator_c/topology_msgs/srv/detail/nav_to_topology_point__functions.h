// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from topology_msgs:srv/NavToTopologyPoint.idl
// generated code does not contain a copyright notice

#ifndef TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__FUNCTIONS_H_
#define TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "topology_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "topology_msgs/srv/detail/nav_to_topology_point__struct.h"

/// Initialize srv/NavToTopologyPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * topology_msgs__srv__NavToTopologyPoint_Request
 * )) before or use
 * topology_msgs__srv__NavToTopologyPoint_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Request__init(topology_msgs__srv__NavToTopologyPoint_Request * msg);

/// Finalize srv/NavToTopologyPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Request__fini(topology_msgs__srv__NavToTopologyPoint_Request * msg);

/// Create srv/NavToTopologyPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * topology_msgs__srv__NavToTopologyPoint_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
topology_msgs__srv__NavToTopologyPoint_Request *
topology_msgs__srv__NavToTopologyPoint_Request__create();

/// Destroy srv/NavToTopologyPoint message.
/**
 * It calls
 * topology_msgs__srv__NavToTopologyPoint_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Request__destroy(topology_msgs__srv__NavToTopologyPoint_Request * msg);

/// Check for srv/NavToTopologyPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Request__are_equal(const topology_msgs__srv__NavToTopologyPoint_Request * lhs, const topology_msgs__srv__NavToTopologyPoint_Request * rhs);

/// Copy a srv/NavToTopologyPoint message.
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
topology_msgs__srv__NavToTopologyPoint_Request__copy(
  const topology_msgs__srv__NavToTopologyPoint_Request * input,
  topology_msgs__srv__NavToTopologyPoint_Request * output);

/// Initialize array of srv/NavToTopologyPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * topology_msgs__srv__NavToTopologyPoint_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__init(topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array, size_t size);

/// Finalize array of srv/NavToTopologyPoint messages.
/**
 * It calls
 * topology_msgs__srv__NavToTopologyPoint_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__fini(topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array);

/// Create array of srv/NavToTopologyPoint messages.
/**
 * It allocates the memory for the array and calls
 * topology_msgs__srv__NavToTopologyPoint_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
topology_msgs__srv__NavToTopologyPoint_Request__Sequence *
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__create(size_t size);

/// Destroy array of srv/NavToTopologyPoint messages.
/**
 * It calls
 * topology_msgs__srv__NavToTopologyPoint_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__destroy(topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array);

/// Check for srv/NavToTopologyPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__are_equal(const topology_msgs__srv__NavToTopologyPoint_Request__Sequence * lhs, const topology_msgs__srv__NavToTopologyPoint_Request__Sequence * rhs);

/// Copy an array of srv/NavToTopologyPoint messages.
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
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__copy(
  const topology_msgs__srv__NavToTopologyPoint_Request__Sequence * input,
  topology_msgs__srv__NavToTopologyPoint_Request__Sequence * output);

/// Initialize srv/NavToTopologyPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * topology_msgs__srv__NavToTopologyPoint_Response
 * )) before or use
 * topology_msgs__srv__NavToTopologyPoint_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Response__init(topology_msgs__srv__NavToTopologyPoint_Response * msg);

/// Finalize srv/NavToTopologyPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Response__fini(topology_msgs__srv__NavToTopologyPoint_Response * msg);

/// Create srv/NavToTopologyPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * topology_msgs__srv__NavToTopologyPoint_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
topology_msgs__srv__NavToTopologyPoint_Response *
topology_msgs__srv__NavToTopologyPoint_Response__create();

/// Destroy srv/NavToTopologyPoint message.
/**
 * It calls
 * topology_msgs__srv__NavToTopologyPoint_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Response__destroy(topology_msgs__srv__NavToTopologyPoint_Response * msg);

/// Check for srv/NavToTopologyPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Response__are_equal(const topology_msgs__srv__NavToTopologyPoint_Response * lhs, const topology_msgs__srv__NavToTopologyPoint_Response * rhs);

/// Copy a srv/NavToTopologyPoint message.
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
topology_msgs__srv__NavToTopologyPoint_Response__copy(
  const topology_msgs__srv__NavToTopologyPoint_Response * input,
  topology_msgs__srv__NavToTopologyPoint_Response * output);

/// Initialize array of srv/NavToTopologyPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * topology_msgs__srv__NavToTopologyPoint_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__init(topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array, size_t size);

/// Finalize array of srv/NavToTopologyPoint messages.
/**
 * It calls
 * topology_msgs__srv__NavToTopologyPoint_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__fini(topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array);

/// Create array of srv/NavToTopologyPoint messages.
/**
 * It allocates the memory for the array and calls
 * topology_msgs__srv__NavToTopologyPoint_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
topology_msgs__srv__NavToTopologyPoint_Response__Sequence *
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__create(size_t size);

/// Destroy array of srv/NavToTopologyPoint messages.
/**
 * It calls
 * topology_msgs__srv__NavToTopologyPoint_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
void
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__destroy(topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array);

/// Check for srv/NavToTopologyPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_topology_msgs
bool
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__are_equal(const topology_msgs__srv__NavToTopologyPoint_Response__Sequence * lhs, const topology_msgs__srv__NavToTopologyPoint_Response__Sequence * rhs);

/// Copy an array of srv/NavToTopologyPoint messages.
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
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__copy(
  const topology_msgs__srv__NavToTopologyPoint_Response__Sequence * input,
  topology_msgs__srv__NavToTopologyPoint_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TOPOLOGY_MSGS__SRV__DETAIL__NAV_TO_TOPOLOGY_POINT__FUNCTIONS_H_
