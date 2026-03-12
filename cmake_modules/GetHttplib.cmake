include(FetchContent)

FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib
    GIT_TAG v0.14.3
    GIT_SHALLOW TRUE)

FetchContent_GetProperties(httplib)
if(NOT httplib_POPULATED)
  FetchContent_Populate(httplib)
  add_library(httplib INTERFACE)
  target_include_directories(httplib INTERFACE ${httplib_SOURCE_DIR})
  add_library(httplib::httplib ALIAS httplib)
endif()
