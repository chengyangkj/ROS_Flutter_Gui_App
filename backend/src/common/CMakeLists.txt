find_package(Boost REQUIRED COMPONENTS filesystem)

add_library(common STATIC
  logger/logger.cc
  logger/easylogging++.cc
  config/config.cpp
)

target_include_directories(common PUBLIC
  ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(common PUBLIC fmt::fmt-header-only Boost::filesystem)

