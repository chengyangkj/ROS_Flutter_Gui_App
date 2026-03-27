include(${CMAKE_SOURCE_DIR}/cmake/GetProtobuf.cmake)

set(PROTO_ROOT "${CMAKE_SOURCE_DIR}/../protocol")
file(GLOB_RECURSE PROTO_FILES CONFIGURE_DEPENDS
  "${PROTO_ROOT}/*.proto"
)
list(SORT PROTO_FILES)

set(PROTO_GEN_SRCS)
set(PROTO_GEN_HDRS)
foreach(proto ${PROTO_FILES})
  get_filename_component(_proto_name "${proto}" NAME_WE)
  list(APPEND PROTO_GEN_SRCS "${CMAKE_CURRENT_BINARY_DIR}/${_proto_name}.pb.cc")
  list(APPEND PROTO_GEN_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${_proto_name}.pb.h")
endforeach()

add_custom_command(
  OUTPUT ${PROTO_GEN_SRCS} ${PROTO_GEN_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
          -I${PROTO_ROOT}
          --experimental_allow_proto3_optional
          --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
          ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating protobuf cpp from protocol/"
  VERBATIM)

add_library(robot_proto STATIC ${PROTO_GEN_SRCS})
target_include_directories(robot_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(robot_proto PUBLIC protobuf::libprotobuf)
