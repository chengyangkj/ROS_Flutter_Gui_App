set(TARGET_NAME ros_gui_backend)

add_executable(${TARGET_NAME}
  main.cpp
  application.cpp
)

target_compile_definitions(${TARGET_NAME} PRIVATE ROS_VERSION=${ROS_VERSION})

target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

if(ROS_VERSION EQUAL 1)
  target_include_directories(${TARGET_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
  map
  web_server
  robot_stream
  ros_gui_node
  Boost::filesystem
  yaml-cpp::yaml-cpp
)

install(TARGETS ${TARGET_NAME} DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/install/bin/
  DESTINATION bin
  USE_SOURCE_PERMISSIONS)

set(ROS_GUI_BACKEND_PREFIX "${CMAKE_INSTALL_PREFIX}")

