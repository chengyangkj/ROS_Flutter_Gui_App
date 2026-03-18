add_executable(map_manager main.cpp map_manager.cpp)

target_compile_definitions(map_manager PRIVATE ROS_VERSION=${ROS_VERSION})

target_include_directories(map_manager PRIVATE ${CMAKE_SOURCE_DIR}/src)

if(ROS_VERSION EQUAL 1)
  target_include_directories(map_manager PRIVATE ${catkin_INCLUDE_DIRS})
endif()

target_link_libraries(map_manager PRIVATE
  map_io
  common
  map_server_node
  Boost::program_options
  Boost::filesystem
  httplib::httplib
  opencv_core
  opencv_imgcodecs
  ${SDL2_DEPS}
  yaml-cpp::yaml-cpp
)

install(TARGETS map_manager DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/install/bin/
    DESTINATION bin
    USE_SOURCE_PERMISSIONS)
