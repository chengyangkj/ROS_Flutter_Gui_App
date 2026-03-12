cmake_minimum_required(VERSION 3.5)
project(nav2_map_server)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

if(NOT DEFINED ROS_VERSION)
  set(ROS_VERSION 2)
endif()

include(${CMAKE_SOURCE_DIR}/cmake_modules/GetYamlCpp.cmake)
include(${CMAKE_SOURCE_DIR}/cmake_modules/GetHttplib.cmake)

find_package(Boost REQUIRED program_options filesystem)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2IMAGE REQUIRED SDL2_image)
add_library(sdl2_map_server INTERFACE)
get_filename_component(SDL2_PARENT_INCLUDE "${SDL2_INCLUDE_DIRS}" DIRECTORY)
target_include_directories(sdl2_map_server INTERFACE
  ${SDL2_PARENT_INCLUDE} ${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})
target_link_libraries(sdl2_map_server INTERFACE
  ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES})
target_compile_options(sdl2_map_server INTERFACE
  ${SDL2_CFLAGS_OTHER} ${SDL2IMAGE_CFLAGS_OTHER})
set(SDL2_DEPS sdl2_map_server)

add_subdirectory(src/common)
add_subdirectory(src/core)
add_subdirectory(src/node)
add_subdirectory(src/app)

