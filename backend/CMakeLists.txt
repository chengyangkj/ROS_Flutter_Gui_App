cmake_minimum_required(VERSION 3.16)
project(ros_gui_backend VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

if(NOT DEFINED ROS_VERSION)
  set(ROS_VERSION 2)
endif()

include(${CMAKE_SOURCE_DIR}/cmake/GetYamlCpp.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GetFmt.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GetDrogon.cmake)

find_package(Boost REQUIRED filesystem)
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
add_subdirectory(src/proto)
add_subdirectory(src/core)
add_subdirectory(src/node)
add_subdirectory(src/core/web_server)
add_subdirectory(src/app)

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated" "${CMAKE_BINARY_DIR}/cpack")
configure_file(
  "${CMAKE_SOURCE_DIR}/src/app/install/bin/service/ros_gui_backend.system.service.in"
  "${CMAKE_BINARY_DIR}/generated/ros_gui_backend.service"
  @ONLY)
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/cpack/postinst.in"
  "${CMAKE_BINARY_DIR}/cpack/postinst"
  COPYONLY)
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake/cpack/prerm.in"
  "${CMAKE_BINARY_DIR}/cpack/prerm"
  COPYONLY)
execute_process(COMMAND chmod 755
  "${CMAKE_BINARY_DIR}/cpack/postinst"
  "${CMAKE_BINARY_DIR}/cpack/prerm")
install(FILES "${CMAKE_BINARY_DIR}/generated/ros_gui_backend.service"
  DESTINATION lib/systemd/system)

get_filename_component(_ROS_GUI_GIT_ROOT "${CMAKE_SOURCE_DIR}" DIRECTORY)
execute_process(
  COMMAND git describe --tags --always --dirty
  WORKING_DIRECTORY "${_ROS_GUI_GIT_ROOT}"
  OUTPUT_VARIABLE _ROS_GUI_GIT_DESCRIBE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
  RESULT_VARIABLE _ROS_GUI_GIT_RC)
if(_ROS_GUI_GIT_RC EQUAL 0 AND NOT _ROS_GUI_GIT_DESCRIBE STREQUAL "")
  string(REGEX REPLACE "^v" "" CPACK_PACKAGE_VERSION "${_ROS_GUI_GIT_DESCRIBE}")
else()
  set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
endif()

# 生成 DEB: cmake -B build -DCMAKE_INSTALL_PREFIX=/usr && cmake --build build --target package
# 二进制链接了 ROS/OpenCV 等时，请在 CPACK_DEBIAN_PACKAGE_DEPENDS 中补全目标机上的运行依赖。
set(CPACK_PACKAGE_NAME "ros-gui-backend")
set(CPACK_PACKAGE_VENDOR "ros-gui-backend")
set(CPACK_PACKAGE_CONTACT "ros-gui-backend@localhost")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ROS GUI Flutter web backend (map, topics, Web UI)")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "${CPACK_PACKAGE_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "ROS GUI Backend <ros-gui-backend@localhost>")
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.28), libstdc++6, libsdl2-2.0-0")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
  "${CMAKE_BINARY_DIR}/cpack/postinst"
  "${CMAKE_BINARY_DIR}/cpack/prerm")
if(UNIX AND NOT APPLE)
  set(CPACK_GENERATOR "DEB")
else()
  set(CPACK_GENERATOR "TGZ")
endif()
include(CPack)

