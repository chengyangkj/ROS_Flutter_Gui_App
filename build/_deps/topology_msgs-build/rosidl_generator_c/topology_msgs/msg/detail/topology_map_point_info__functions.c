// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from topology_msgs:msg/TopologyMapPointInfo.idl
// generated code does not contain a copyright notice
#include "topology_msgs/msg/detail/topology_map_point_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
topology_msgs__msg__TopologyMapPointInfo__init(topology_msgs__msg__TopologyMapPointInfo * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    topology_msgs__msg__TopologyMapPointInfo__fini(msg);
    return false;
  }
  // x
  // y
  // theta
  // type
  return true;
}

void
topology_msgs__msg__TopologyMapPointInfo__fini(topology_msgs__msg__TopologyMapPointInfo * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // x
  // y
  // theta
  // type
}

bool
topology_msgs__msg__TopologyMapPointInfo__are_equal(const topology_msgs__msg__TopologyMapPointInfo * lhs, const topology_msgs__msg__TopologyMapPointInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  return true;
}

bool
topology_msgs__msg__TopologyMapPointInfo__copy(
  const topology_msgs__msg__TopologyMapPointInfo * input,
  topology_msgs__msg__TopologyMapPointInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  // type
  output->type = input->type;
  return true;
}

topology_msgs__msg__TopologyMapPointInfo *
topology_msgs__msg__TopologyMapPointInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__TopologyMapPointInfo * msg = (topology_msgs__msg__TopologyMapPointInfo *)allocator.allocate(sizeof(topology_msgs__msg__TopologyMapPointInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topology_msgs__msg__TopologyMapPointInfo));
  bool success = topology_msgs__msg__TopologyMapPointInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topology_msgs__msg__TopologyMapPointInfo__destroy(topology_msgs__msg__TopologyMapPointInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topology_msgs__msg__TopologyMapPointInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topology_msgs__msg__TopologyMapPointInfo__Sequence__init(topology_msgs__msg__TopologyMapPointInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__TopologyMapPointInfo * data = NULL;

  if (size) {
    data = (topology_msgs__msg__TopologyMapPointInfo *)allocator.zero_allocate(size, sizeof(topology_msgs__msg__TopologyMapPointInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topology_msgs__msg__TopologyMapPointInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topology_msgs__msg__TopologyMapPointInfo__fini(&data[i - 1]);
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
topology_msgs__msg__TopologyMapPointInfo__Sequence__fini(topology_msgs__msg__TopologyMapPointInfo__Sequence * array)
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
      topology_msgs__msg__TopologyMapPointInfo__fini(&array->data[i]);
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

topology_msgs__msg__TopologyMapPointInfo__Sequence *
topology_msgs__msg__TopologyMapPointInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__TopologyMapPointInfo__Sequence * array = (topology_msgs__msg__TopologyMapPointInfo__Sequence *)allocator.allocate(sizeof(topology_msgs__msg__TopologyMapPointInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topology_msgs__msg__TopologyMapPointInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topology_msgs__msg__TopologyMapPointInfo__Sequence__destroy(topology_msgs__msg__TopologyMapPointInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topology_msgs__msg__TopologyMapPointInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topology_msgs__msg__TopologyMapPointInfo__Sequence__are_equal(const topology_msgs__msg__TopologyMapPointInfo__Sequence * lhs, const topology_msgs__msg__TopologyMapPointInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topology_msgs__msg__TopologyMapPointInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topology_msgs__msg__TopologyMapPointInfo__Sequence__copy(
  const topology_msgs__msg__TopologyMapPointInfo__Sequence * input,
  topology_msgs__msg__TopologyMapPointInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topology_msgs__msg__TopologyMapPointInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topology_msgs__msg__TopologyMapPointInfo * data =
      (topology_msgs__msg__TopologyMapPointInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topology_msgs__msg__TopologyMapPointInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topology_msgs__msg__TopologyMapPointInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topology_msgs__msg__TopologyMapPointInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
