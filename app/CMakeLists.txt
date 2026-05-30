cmake_minimum_required(VERSION 3.5)

project(rs_driver_main)

include_directories(${DRIVER_INCLUDE_DIRS})

add_executable(main
               main.cpp)

target_link_libraries(main
                      ${EXTERNAL_LIBS})
