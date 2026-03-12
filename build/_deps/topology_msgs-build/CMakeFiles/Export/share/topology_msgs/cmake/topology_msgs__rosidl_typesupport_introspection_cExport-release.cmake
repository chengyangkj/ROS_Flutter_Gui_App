#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "topology_msgs::topology_msgs__rosidl_typesupport_introspection_c" for configuration "Release"
set_property(TARGET topology_msgs::topology_msgs__rosidl_typesupport_introspection_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(topology_msgs::topology_msgs__rosidl_typesupport_introspection_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtopology_msgs__rosidl_typesupport_introspection_c.so"
  IMPORTED_SONAME_RELEASE "libtopology_msgs__rosidl_typesupport_introspection_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS topology_msgs::topology_msgs__rosidl_typesupport_introspection_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_topology_msgs::topology_msgs__rosidl_typesupport_introspection_c "${_IMPORT_PREFIX}/lib/libtopology_msgs__rosidl_typesupport_introspection_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
