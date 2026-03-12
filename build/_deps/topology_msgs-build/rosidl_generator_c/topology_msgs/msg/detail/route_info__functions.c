// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from topology_msgs:msg/RouteInfo.idl
// generated code does not contain a copyright notice
#include "topology_msgs/msg/detail/route_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `controller`
// Member `goal_checker`
#include "rosidl_runtime_c/string_functions.h"

bool
topology_msgs__msg__RouteInfo__init(topology_msgs__msg__RouteInfo * msg)
{
  if (!msg) {
    return false;
  }
  // controller
  if (!rosidl_runtime_c__String__init(&msg->controller)) {
    topology_msgs__msg__RouteInfo__fini(msg);
    return false;
  }
  // goal_checker
  if (!rosidl_runtime_c__String__init(&msg->goal_checker)) {
    topology_msgs__msg__RouteInfo__fini(msg);
    return false;
  }
  // speed_limit
  return true;
}

void
topology_msgs__msg__RouteInfo__fini(topology_msgs__msg__RouteInfo * msg)
{
  if (!msg) {
    return;
  }
  // controller
  rosidl_runtime_c__String__fini(&msg->controller);
  // goal_checker
  rosidl_runtime_c__String__fini(&msg->goal_checker);
  // speed_limit
}

bool
topology_msgs__msg__RouteInfo__are_equal(const topology_msgs__msg__RouteInfo * lhs, const topology_msgs__msg__RouteInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // controller
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->controller), &(rhs->controller)))
  {
    return false;
  }
  // goal_checker
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->goal_checker), &(rhs->goal_checker)))
  {
    return false;
  }
  // speed_limit
  if (lhs->speed_limit != rhs->speed_limit) {
    return false;
  }
  return true;
}

bool
topology_msgs__msg__RouteInfo__copy(
  const topology_msgs__msg__RouteInfo * input,
  topology_msgs__msg__RouteInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // controller
  if (!rosidl_runtime_c__String__copy(
      &(input->controller), &(output->controller)))
  {
    return false;
  }
  // goal_checker
  if (!rosidl_runtime_c__String__copy(
      &(input->goal_checker), &(output->goal_checker)))
  {
    return false;
  }
  // speed_limit
  output->speed_limit = input->speed_limit;
  return true;
}

topology_msgs__msg__RouteInfo *
topology_msgs__msg__RouteInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__RouteInfo * msg = (topology_msgs__msg__RouteInfo *)allocator.allocate(sizeof(topology_msgs__msg__RouteInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topology_msgs__msg__RouteInfo));
  bool success = topology_msgs__msg__RouteInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topology_msgs__msg__RouteInfo__destroy(topology_msgs__msg__RouteInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topology_msgs__msg__RouteInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topology_msgs__msg__RouteInfo__Sequence__init(topology_msgs__msg__RouteInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__RouteInfo * data = NULL;

  if (size) {
    data = (topology_msgs__msg__RouteInfo *)allocator.zero_allocate(size, sizeof(topology_msgs__msg__RouteInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topology_msgs__msg__RouteInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topology_msgs__msg__RouteInfo__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
topology_msgs__msg__RouteInfo__Sequence__fini(topology_msgs__msg__RouteInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      topology_msgs__msg__RouteInfo__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

topology_msgs__msg__RouteInfo__Sequence *
topology_msgs__msg__RouteInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__RouteInfo__Sequence * array = (topology_msgs__msg__RouteInfo__Sequence *)allocator.allocate(sizeof(topology_msgs__msg__RouteInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topology_msgs__msg__RouteInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topology_msgs__msg__RouteInfo__Sequence__destroy(topology_msgs__msg__RouteInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topology_msgs__msg__RouteInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topology_msgs__msg__RouteInfo__Sequence__are_equal(const topology_msgs__msg__RouteInfo__Sequence * lhs, const topology_msgs__msg__RouteInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topology_msgs__msg__RouteInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topology_msgs__msg__RouteInfo__Sequence__copy(
  const topology_msgs__msg__RouteInfo__Sequence * input,
  topology_msgs__msg__RouteInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topology_msgs__msg__RouteInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topology_msgs__msg__RouteInfo * data =
      (topology_msgs__msg__RouteInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topology_msgs__msg__RouteInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topology_msgs__msg__RouteInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topology_msgs__msg__RouteInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
