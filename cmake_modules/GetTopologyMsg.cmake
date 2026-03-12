include(FetchContent)

message(STATUS "get topology_msgs ...")

set(topology_msgs_GIT_REPOSITORY
    "https://gitee.com/chengyangkj/topology_msgs.git"
    CACHE STRING "")

FetchContent_Declare(
  topology_msgs
  GIT_REPOSITORY ${topology_msgs_GIT_REPOSITORY}
  GIT_TAG "main"
  GIT_SHALLOW TRUE)

FetchContent_GetProperties(topology_msgs)
if(NOT topology_msgs_POPULATED)
  FetchContent_MakeAvailable(topology_msgs)
  
  set(TOPOLOGY_MSGS_TARGETS
    topology_msgs__rosidl_generator_cpp
    topology_msgs__rosidl_typesupport_cpp
    topology_msgs__rosidl_typesupport_fastrtps_cpp
    topology_msgs__rosidl_typesupport_introspection_cpp
  )
  
  foreach(TARGET_NAME ${TOPOLOGY_MSGS_TARGETS})
    if(TARGET ${TARGET_NAME})
      set_target_properties(${TARGET_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
      )
      
      install(TARGETS ${TARGET_NAME}
        RUNTIME DESTINATION bin/lib
        LIBRARY DESTINATION bin/lib
        ARCHIVE DESTINATION bin/lib
      )
    endif()
  endforeach()
endif()

# import targets:
# topology_msgs::topology_msgs 