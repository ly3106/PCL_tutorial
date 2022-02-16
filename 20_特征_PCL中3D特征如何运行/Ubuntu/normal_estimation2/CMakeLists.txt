cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(normal_estimation2)
find_package(PCL 1.3 REQUIRED COMPONENTS common io features visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(normal_estimation2 normal_estimation2.cpp)
target_link_libraries(normal_estimation2 ${PCL_LIBRARIES})


