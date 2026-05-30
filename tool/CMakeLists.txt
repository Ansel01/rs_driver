
cmake_minimum_required(VERSION 3.5)

project(rs_driver_viewer)

message(=============================================================)
message("-- Ready to compile tools")
message(=============================================================)

include_directories(${DRIVER_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(${COMPILE_TOOL_VIEWER})

find_package(PCL COMPONENTS common visualization io QUIET REQUIRED)
# find_package(PCL 1.14  COMPONENTS common visualization io QUIET REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

if(PCL_FOUND)

add_executable(rs_driver_viewer
               rs_driver_viewer.cpp)

target_link_libraries(rs_driver_viewer
                    ${EXTERNAL_LIBS}    
                    ${PCL_LIBRARIES})

else()

message("PCL Not found! Can not compile rs_driver_viewer!")

endif()

install(TARGETS rs_driver_viewer
        RUNTIME DESTINATION /usr/bin)     

endif (${COMPILE_TOOL_VIEWER})


if(${COMPILE_TOOL_PCDSAVER})

add_executable(rs_driver_pcdsaver
               rs_driver_pcdsaver.cpp)

target_link_libraries(rs_driver_pcdsaver
                    ${EXTERNAL_LIBS})

endif(${COMPILE_TOOL_PCDSAVER})
