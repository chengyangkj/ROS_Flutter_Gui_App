include(FetchContent)
message(STATUS "download drgon ...")

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_CTL OFF CACHE BOOL "" FORCE)
set(BUILD_ORM OFF CACHE BOOL "" FORCE)

set(Drogon_GIT_REPOSITORY https://github.com/drogonframework/drogon.git CACHE STRING "drogon git repository")

FetchContent_Declare(
  drogon
  GIT_REPOSITORY https://github.com/drogonframework/drogon.git
  GIT_TAG v1.9.11
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(drogon)
