cmake_minimum_required(VERSION 2.8.12)

project(main)

find_package(OpenCV REQUIRED)
find_package(Threads)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries(main ${CMAKE_THREAD_LIBS_INIT})
