add_executable(map_server main.cpp)

target_compile_definitions(map_server PRIVATE ROS_VERSION=${ROS_VERSION})

target_include_directories(map_server PRIVATE ${CMAKE_SOURCE_DIR}/src)

if(ROS_VERSION EQUAL 1)
  target_include_directories(map_server PRIVATE ${catkin_INCLUDE_DIRS})
endif()

target_link_libraries(map_server PRIVATE map_server_node common Boost::program_options)

install(TARGETS map_server DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/install/bin/
    DESTINATION bin
    USE_SOURCE_PERMISSIONS)
