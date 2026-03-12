add_library(common STATIC
  logger/logger.cc
  logger/easylogging++.cc
)

target_include_directories(common PUBLIC
  ${CMAKE_SOURCE_DIR}/src
)

