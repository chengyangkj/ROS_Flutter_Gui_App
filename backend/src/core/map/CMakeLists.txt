add_library(map STATIC
  map_mode.cpp
  map_io.cpp
  tiles_map_generator.cc
  map_manager.cpp
)

target_include_directories(map PUBLIC ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(map PUBLIC
  common
  Boost::filesystem
  opencv_core
  opencv_imgcodecs
  ${SDL2_DEPS}
  yaml-cpp::yaml-cpp
)
