include(${CMAKE_SOURCE_DIR}/cmake_modules/GetTopologyMsg.cmake)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(map_server_node STATIC map_server_node.cpp)
target_compile_definitions(map_server_node PRIVATE ROS_VERSION=2)
target_include_directories(map_server_node PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(map_server_node PUBLIC
    Boost::filesystem
    map_io
    rclcpp::rclcpp
    ${nav2_msgs_TARGETS}
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    topology_msgs__rosidl_generator_cpp #消息类型的ROS库
    topology_msgs__rosidl_typesupport_cpp
    topology_msgs__rosidl_typesupport_fastrtps_cpp
    topology_msgs__rosidl_typesupport_introspection_cpp
)
