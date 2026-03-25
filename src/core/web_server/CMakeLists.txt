add_library(web_server STATIC web_server.cpp)

target_include_directories(web_server PUBLIC ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(web_server PUBLIC map drogon common)
