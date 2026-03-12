// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from topology_msgs:srv/NavToTopologyPoint.idl
// generated code does not contain a copyright notice
#include "topology_msgs/srv/detail/nav_to_topology_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `point_name`
#include "rosidl_runtime_c/string_functions.h"

bool
topology_msgs__srv__NavToTopologyPoint_Request__init(topology_msgs__srv__NavToTopologyPoint_Request * msg)
{
  if (!msg) {
    return false;
  }
  // point_name
  if (!rosidl_runtime_c__String__init(&msg->point_name)) {
    topology_msgs__srv__NavToTopologyPoint_Request__fini(msg);
    return false;
  }
  return true;
}

void
topology_msgs__srv__NavToTopologyPoint_Request__fini(topology_msgs__srv__NavToTopologyPoint_Request * msg)
{
  if (!msg) {
    return;
  }
  // point_name
  rosidl_runtime_c__String__fini(&msg->point_name);
}

bool
topology_msgs__srv__NavToTopologyPoint_Request__are_equal(const topology_msgs__srv__NavToTopologyPoint_Request * lhs, const topology_msgs__srv__NavToTopologyPoint_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // point_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->point_name), &(rhs->point_name)))
  {
    return false;
  }
  return true;
}

bool
topology_msgs__srv__NavToTopologyPoint_Request__copy(
  const topology_msgs__srv__NavToTopologyPoint_Request * input,
  topology_msgs__srv__NavToTopologyPoint_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // point_name
  if (!rosidl_runtime_c__String__copy(
      &(input->point_name), &(output->point_name)))
  {
    return false;
  }
  return true;
}

topology_msgs__srv__NavToTopologyPoint_Request *
topology_msgs__srv__NavToTopologyPoint_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__srv__NavToTopologyPoint_Request * msg = (topology_msgs__srv__NavToTopologyPoint_Request *)allocator.allocate(sizeof(topology_msgs__srv__NavToTopologyPoint_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topology_msgs__srv__NavToTopologyPoint_Request));
  bool success = topology_msgs__srv__NavToTopologyPoint_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topology_msgs__srv__NavToTopologyPoint_Request__destroy(topology_msgs__srv__NavToTopologyPoint_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topology_msgs__srv__NavToTopologyPoint_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__init(topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__srv__NavToTopologyPoint_Request * data = NULL;

  if (size) {
    data = (topology_msgs__srv__NavToTopologyPoint_Request *)allocator.zero_allocate(size, sizeof(topology_msgs__srv__NavToTopologyPoint_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topology_msgs__srv__NavToTopologyPoint_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topology_msgs__srv__NavToTopologyPoint_Request__fini(&data[i - 1]);
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
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__fini(topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array)
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
      topology_msgs__srv__NavToTopologyPoint_Request__fini(&array->data[i]);
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

topology_msgs__srv__NavToTopologyPoint_Request__Sequence *
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array = (topology_msgs__srv__NavToTopologyPoint_Request__Sequence *)allocator.allocate(sizeof(topology_msgs__srv__NavToTopologyPoint_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topology_msgs__srv__NavToTopologyPoint_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__destroy(topology_msgs__srv__NavToTopologyPoint_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topology_msgs__srv__NavToTopologyPoint_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__are_equal(const topology_msgs__srv__NavToTopologyPoint_Request__Sequence * lhs, const topology_msgs__srv__NavToTopologyPoint_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topology_msgs__srv__NavToTopologyPoint_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topology_msgs__srv__NavToTopologyPoint_Request__Sequence__copy(
  const topology_msgs__srv__NavToTopologyPoint_Request__Sequence * input,
  topology_msgs__srv__NavToTopologyPoint_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topology_msgs__srv__NavToTopologyPoint_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topology_msgs__srv__NavToTopologyPoint_Request * data =
      (topology_msgs__srv__NavToTopologyPoint_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topology_msgs__srv__NavToTopologyPoint_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topology_msgs__srv__NavToTopologyPoint_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topology_msgs__srv__NavToTopologyPoint_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `task_id`
// Member `msg`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
topology_msgs__srv__NavToTopologyPoint_Response__init(topology_msgs__srv__NavToTopologyPoint_Response * msg)
{
  if (!msg) {
    return false;
  }
  // is_success
  // task_id
  if (!rosidl_runtime_c__String__init(&msg->task_id)) {
    topology_msgs__srv__NavToTopologyPoint_Response__fini(msg);
    return false;
  }
  // msg
  if (!rosidl_runtime_c__String__init(&msg->msg)) {
    topology_msgs__srv__NavToTopologyPoint_Response__fini(msg);
    return false;
  }
  return true;
}

void
topology_msgs__srv__NavToTopologyPoint_Response__fini(topology_msgs__srv__NavToTopologyPoint_Response * msg)
{
  if (!msg) {
    return;
  }
  // is_success
  // task_id
  rosidl_runtime_c__String__fini(&msg->task_id);
  // msg
  rosidl_runtime_c__String__fini(&msg->msg);
}

bool
topology_msgs__srv__NavToTopologyPoint_Response__are_equal(const topology_msgs__srv__NavToTopologyPoint_Response * lhs, const topology_msgs__srv__NavToTopologyPoint_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_success
  if (lhs->is_success != rhs->is_success) {
    return false;
  }
  // task_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_id), &(rhs->task_id)))
  {
    return false;
  }
  // msg
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->msg), &(rhs->msg)))
  {
    return false;
  }
  return true;
}

bool
topology_msgs__srv__NavToTopologyPoint_Response__copy(
  const topology_msgs__srv__NavToTopologyPoint_Response * input,
  topology_msgs__srv__NavToTopologyPoint_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // is_success
  output->is_success = input->is_success;
  // task_id
  if (!rosidl_runtime_c__String__copy(
      &(input->task_id), &(output->task_id)))
  {
    return false;
  }
  // msg
  if (!rosidl_runtime_c__String__copy(
      &(input->msg), &(output->msg)))
  {
    return false;
  }
  return true;
}

topology_msgs__srv__NavToTopologyPoint_Response *
topology_msgs__srv__NavToTopologyPoint_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__srv__NavToTopologyPoint_Response * msg = (topology_msgs__srv__NavToTopologyPoint_Response *)allocator.allocate(sizeof(topology_msgs__srv__NavToTopologyPoint_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topology_msgs__srv__NavToTopologyPoint_Response));
  bool success = topology_msgs__srv__NavToTopologyPoint_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topology_msgs__srv__NavToTopologyPoint_Response__destroy(topology_msgs__srv__NavToTopologyPoint_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topology_msgs__srv__NavToTopologyPoint_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__init(topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__srv__NavToTopologyPoint_Response * data = NULL;

  if (size) {
    data = (topology_msgs__srv__NavToTopologyPoint_Response *)allocator.zero_allocate(size, sizeof(topology_msgs__srv__NavToTopologyPoint_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topology_msgs__srv__NavToTopologyPoint_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topology_msgs__srv__NavToTopologyPoint_Response__fini(&data[i - 1]);
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
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__fini(topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array)
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
      topology_msgs__srv__NavToTopologyPoint_Response__fini(&array->data[i]);
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

topology_msgs__srv__NavToTopologyPoint_Response__Sequence *
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array = (topology_msgs__srv__NavToTopologyPoint_Response__Sequence *)allocator.allocate(sizeof(topology_msgs__srv__NavToTopologyPoint_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topology_msgs__srv__NavToTopologyPoint_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__destroy(topology_msgs__srv__NavToTopologyPoint_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topology_msgs__srv__NavToTopologyPoint_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__are_equal(const topology_msgs__srv__NavToTopologyPoint_Response__Sequence * lhs, const topology_msgs__srv__NavToTopologyPoint_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topology_msgs__srv__NavToTopologyPoint_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topology_msgs__srv__NavToTopologyPoint_Response__Sequence__copy(
  const topology_msgs__srv__NavToTopologyPoint_Response__Sequence * input,
  topology_msgs__srv__NavToTopologyPoint_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topology_msgs__srv__NavToTopologyPoint_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topology_msgs__srv__NavToTopologyPoint_Response * data =
      (topology_msgs__srv__NavToTopologyPoint_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topology_msgs__srv__NavToTopologyPoint_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topology_msgs__srv__NavToTopologyPoint_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topology_msgs__srv__NavToTopologyPoint_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
