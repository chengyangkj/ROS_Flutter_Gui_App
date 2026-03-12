// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice
#include "topology_msgs/msg/detail/topology_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `map_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `map_property`
#include "topology_msgs/msg/detail/map_property__functions.h"
// Member `points`
#include "topology_msgs/msg/detail/topology_map_point_info__functions.h"
// Member `routes`
#include "topology_msgs/msg/detail/route_connection__functions.h"

bool
topology_msgs__msg__TopologyMap__init(topology_msgs__msg__TopologyMap * msg)
{
  if (!msg) {
    return false;
  }
  // map_name
  if (!rosidl_runtime_c__String__init(&msg->map_name)) {
    topology_msgs__msg__TopologyMap__fini(msg);
    return false;
  }
  // map_property
  if (!topology_msgs__msg__MapProperty__init(&msg->map_property)) {
    topology_msgs__msg__TopologyMap__fini(msg);
    return false;
  }
  // points
  if (!topology_msgs__msg__TopologyMapPointInfo__Sequence__init(&msg->points, 0)) {
    topology_msgs__msg__TopologyMap__fini(msg);
    return false;
  }
  // routes
  if (!topology_msgs__msg__RouteConnection__Sequence__init(&msg->routes, 0)) {
    topology_msgs__msg__TopologyMap__fini(msg);
    return false;
  }
  return true;
}

void
topology_msgs__msg__TopologyMap__fini(topology_msgs__msg__TopologyMap * msg)
{
  if (!msg) {
    return;
  }
  // map_name
  rosidl_runtime_c__String__fini(&msg->map_name);
  // map_property
  topology_msgs__msg__MapProperty__fini(&msg->map_property);
  // points
  topology_msgs__msg__TopologyMapPointInfo__Sequence__fini(&msg->points);
  // routes
  topology_msgs__msg__RouteConnection__Sequence__fini(&msg->routes);
}

bool
topology_msgs__msg__TopologyMap__are_equal(const topology_msgs__msg__TopologyMap * lhs, const topology_msgs__msg__TopologyMap * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // map_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->map_name), &(rhs->map_name)))
  {
    return false;
  }
  // map_property
  if (!topology_msgs__msg__MapProperty__are_equal(
      &(lhs->map_property), &(rhs->map_property)))
  {
    return false;
  }
  // points
  if (!topology_msgs__msg__TopologyMapPointInfo__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // routes
  if (!topology_msgs__msg__RouteConnection__Sequence__are_equal(
      &(lhs->routes), &(rhs->routes)))
  {
    return false;
  }
  return true;
}

bool
topology_msgs__msg__TopologyMap__copy(
  const topology_msgs__msg__TopologyMap * input,
  topology_msgs__msg__TopologyMap * output)
{
  if (!input || !output) {
    return false;
  }
  // map_name
  if (!rosidl_runtime_c__String__copy(
      &(input->map_name), &(output->map_name)))
  {
    return false;
  }
  // map_property
  if (!topology_msgs__msg__MapProperty__copy(
      &(input->map_property), &(output->map_property)))
  {
    return false;
  }
  // points
  if (!topology_msgs__msg__TopologyMapPointInfo__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // routes
  if (!topology_msgs__msg__RouteConnection__Sequence__copy(
      &(input->routes), &(output->routes)))
  {
    return false;
  }
  return true;
}

topology_msgs__msg__TopologyMap *
topology_msgs__msg__TopologyMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__TopologyMap * msg = (topology_msgs__msg__TopologyMap *)allocator.allocate(sizeof(topology_msgs__msg__TopologyMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(topology_msgs__msg__TopologyMap));
  bool success = topology_msgs__msg__TopologyMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
topology_msgs__msg__TopologyMap__destroy(topology_msgs__msg__TopologyMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    topology_msgs__msg__TopologyMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
topology_msgs__msg__TopologyMap__Sequence__init(topology_msgs__msg__TopologyMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__TopologyMap * data = NULL;

  if (size) {
    data = (topology_msgs__msg__TopologyMap *)allocator.zero_allocate(size, sizeof(topology_msgs__msg__TopologyMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = topology_msgs__msg__TopologyMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        topology_msgs__msg__TopologyMap__fini(&data[i - 1]);
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
topology_msgs__msg__TopologyMap__Sequence__fini(topology_msgs__msg__TopologyMap__Sequence * array)
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
      topology_msgs__msg__TopologyMap__fini(&array->data[i]);
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

topology_msgs__msg__TopologyMap__Sequence *
topology_msgs__msg__TopologyMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  topology_msgs__msg__TopologyMap__Sequence * array = (topology_msgs__msg__TopologyMap__Sequence *)allocator.allocate(sizeof(topology_msgs__msg__TopologyMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = topology_msgs__msg__TopologyMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
topology_msgs__msg__TopologyMap__Sequence__destroy(topology_msgs__msg__TopologyMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    topology_msgs__msg__TopologyMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
topology_msgs__msg__TopologyMap__Sequence__are_equal(const topology_msgs__msg__TopologyMap__Sequence * lhs, const topology_msgs__msg__TopologyMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!topology_msgs__msg__TopologyMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
topology_msgs__msg__TopologyMap__Sequence__copy(
  const topology_msgs__msg__TopologyMap__Sequence * input,
  topology_msgs__msg__TopologyMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(topology_msgs__msg__TopologyMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    topology_msgs__msg__TopologyMap * data =
      (topology_msgs__msg__TopologyMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!topology_msgs__msg__TopologyMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          topology_msgs__msg__TopologyMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!topology_msgs__msg__TopologyMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
