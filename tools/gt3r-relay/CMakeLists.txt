# Standalone build of the relay server (no SDL, no game code): for the server machine.
#   cmake -S tools/gt3r-relay -B build-relay -DCMAKE_BUILD_TYPE=Release && cmake --build build-relay
cmake_minimum_required(VERSION 3.21)
project(gt3r_relay LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(GT3R_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
add_executable(gt3r-relay
  main.cpp
  ${GT3R_SRC}/net/socket.cpp
  ${GT3R_SRC}/net/protocol.cpp
  ${GT3R_SRC}/net/rendezvous.cpp
  ${GT3R_SRC}/net/relay_server.cpp)
target_include_directories(gt3r-relay PRIVATE ${GT3R_SRC})
if(WIN32)
  target_link_libraries(gt3r-relay PRIVATE ws2_32)
endif()
install(TARGETS gt3r-relay RUNTIME DESTINATION bin)
# The load test (docs/dev/ONLINE.md "Benchmark"): simulated relayed races against a relay.
if(NOT WIN32)
  add_executable(gt3r-relay-bench
    bench.cpp
    ${GT3R_SRC}/net/socket.cpp
    ${GT3R_SRC}/net/protocol.cpp
    ${GT3R_SRC}/net/rendezvous.cpp)
  target_include_directories(gt3r-relay-bench PRIVATE ${GT3R_SRC})
endif()
