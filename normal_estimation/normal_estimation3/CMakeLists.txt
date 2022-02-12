cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(normal_estimation3)
find_package(PCL 1.3 REQUIRED COMPONENTS common io features visualization filters)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(normal_estimation3 normal_estimation3.cpp)
target_link_libraries(normal_estimation3 ${PCL_LIBRARIES})


