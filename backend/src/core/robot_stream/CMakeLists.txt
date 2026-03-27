add_library(robot_stream STATIC
  robot_message_hub.cpp
  robot_ws_controller.cpp
  ssh_tunnel_ws_controller.cpp
)

target_include_directories(robot_stream PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(robot_stream PUBLIC drogon robot_proto common)
