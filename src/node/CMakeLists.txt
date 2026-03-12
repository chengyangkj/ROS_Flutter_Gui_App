find_package(Boost REQUIRED program_options)

if(ROS_VERSION EQUAL 1)
  add_subdirectory(ros1)
else()
  add_subdirectory(ros2)
endif()
