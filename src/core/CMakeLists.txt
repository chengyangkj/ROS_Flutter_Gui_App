add_library(map_io STATIC
  map_mode.cpp
  map_io.cpp
  tiles_map_generator.cc
)

target_include_directories(map_io PUBLIC
  ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(map_io PUBLIC
  common
  Boost::filesystem
  opencv_core
  opencv_imgcodecs
  httplib::httplib
  ${SDL2_DEPS}
  yaml-cpp::yaml-cpp
)

