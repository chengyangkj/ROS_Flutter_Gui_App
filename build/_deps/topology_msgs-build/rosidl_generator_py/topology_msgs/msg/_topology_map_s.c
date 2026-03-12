// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from topology_msgs:msg/TopologyMap.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "topology_msgs/msg/detail/topology_map__struct.h"
#include "topology_msgs/msg/detail/topology_map__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "topology_msgs/msg/detail/route_connection__functions.h"
#include "topology_msgs/msg/detail/topology_map_point_info__functions.h"
// end nested array functions include
bool topology_msgs__msg__map_property__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * topology_msgs__msg__map_property__convert_to_py(void * raw_ros_message);
bool topology_msgs__msg__topology_map_point_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * topology_msgs__msg__topology_map_point_info__convert_to_py(void * raw_ros_message);
bool topology_msgs__msg__route_connection__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * topology_msgs__msg__route_connection__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool topology_msgs__msg__topology_map__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("topology_msgs.msg._topology_map.TopologyMap", full_classname_dest, 43) == 0);
  }
  topology_msgs__msg__TopologyMap * ros_message = _ros_message;
  {  // map_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "map_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->map_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // map_property
    PyObject * field = PyObject_GetAttrString(_pymsg, "map_property");
    if (!field) {
      return false;
    }
    if (!topology_msgs__msg__map_property__convert_from_py(field, &ros_message->map_property)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // points
    PyObject * field = PyObject_GetAttrString(_pymsg, "points");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'points'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!topology_msgs__msg__TopologyMapPointInfo__Sequence__init(&(ros_message->points), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create topology_msgs__msg__TopologyMapPointInfo__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    topology_msgs__msg__TopologyMapPointInfo * dest = ros_message->points.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!topology_msgs__msg__topology_map_point_info__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // routes
    PyObject * field = PyObject_GetAttrString(_pymsg, "routes");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'routes'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!topology_msgs__msg__RouteConnection__Sequence__init(&(ros_message->routes), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create topology_msgs__msg__RouteConnection__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    topology_msgs__msg__RouteConnection * dest = ros_message->routes.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!topology_msgs__msg__route_connection__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * topology_msgs__msg__topology_map__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TopologyMap */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("topology_msgs.msg._topology_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TopologyMap");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  topology_msgs__msg__TopologyMap * ros_message = (topology_msgs__msg__TopologyMap *)raw_ros_message;
  {  // map_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->map_name.data,
      strlen(ros_message->map_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "map_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // map_property
    PyObject * field = NULL;
    field = topology_msgs__msg__map_property__convert_to_py(&ros_message->map_property);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "map_property", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // points
    PyObject * field = NULL;
    size_t size = ros_message->points.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    topology_msgs__msg__TopologyMapPointInfo * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->points.data[i]);
      PyObject * pyitem = topology_msgs__msg__topology_map_point_info__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "points", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // routes
    PyObject * field = NULL;
    size_t size = ros_message->routes.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    topology_msgs__msg__RouteConnection * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->routes.data[i]);
      PyObject * pyitem = topology_msgs__msg__route_connection__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "routes", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
