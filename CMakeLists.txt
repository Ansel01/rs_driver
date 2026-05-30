cmake_minimum_required(VERSION 3.5)

project(rs_driver VERSION 1.5.19)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This trimmed rs_driver only supports Linux.")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
option(ENABLE_MODIFY_RECVBUF "Enable modify size of RCVBUF" ON)
option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
option(ENABLE_PCL_POINTCLOUD "Enable PCL Point Cloud" OFF)
option(ENABLE_CRC32_CHECK "Enable CRC32 Check on MSOP Packet" OFF)
option(ENABLE_DIFOP_PARSE "Enable parsing DIFOP Packet" OFF)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(COMPILER "Linux GNU Compiler")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILER "Linux Clang Compiler")
else()
  message(FATAL_ERROR "Unsupported compiler. This trimmed rs_driver supports GNU or Clang on Linux.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

message(=============================================================)
message("-- CMake run for ${COMPILER}")
message(=============================================================)

set(DRIVER_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include)
list(APPEND EXTERNAL_LIBS pthread)

if(ENABLE_EPOLL_RECEIVE)
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif()

if(ENABLE_TRANSFORM)
  add_definitions("-DENABLE_TRANSFORM")
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

if(ENABLE_MODIFY_RECVBUF)
  add_definitions("-DENABLE_MODIFY_RECVBUF")
endif()

if(ENABLE_WAIT_IF_QUEUE_EMPTY)
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif()

if(ENABLE_STAMP_WITH_LOCAL)
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif()

if(ENABLE_PCL_POINTCLOUD)
  add_definitions("-DENABLE_PCL_POINTCLOUD")
endif()

if(ENABLE_CRC32_CHECK)
  add_definitions("-DENABLE_CRC32_CHECK")
endif()

if(ENABLE_DIFOP_PARSE)
  add_definitions("-DENABLE_DIFOP_PARSE")
endif()

add_executable(main
  ${CMAKE_CURRENT_LIST_DIR}/app/main.cpp)

target_include_directories(main PRIVATE
  ${DRIVER_INCLUDE_DIRS})

target_link_libraries(main PRIVATE
  ${EXTERNAL_LIBS})
