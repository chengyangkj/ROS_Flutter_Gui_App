find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(ros_gui_node STATIC node.cpp)
target_compile_definitions(ros_gui_node PRIVATE ROS_VERSION=2)
target_include_directories(ros_gui_node PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ros_gui_node PUBLIC
  Boost::filesystem
  map
  rclcpp::rclcpp
  ${nav2_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)
