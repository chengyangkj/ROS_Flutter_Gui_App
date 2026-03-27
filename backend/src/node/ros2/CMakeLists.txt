find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(ros_gui_node STATIC node.cpp node_streamer.cpp ../node_manager.cpp)

target_compile_definitions(ros_gui_node PRIVATE ROS_VERSION=2)

target_include_directories(ros_gui_node PUBLIC ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(ros_gui_node PUBLIC
  Boost::filesystem
  common
  map
  robot_stream
  robot_proto
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  action_msgs::action_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  nav2_msgs::nav2_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  opencv_imgcodecs
  opencv_core
)
