cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(normal_estimation)
find_package(PCL 1.3 REQUIRED COMPONENTS common io features visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(normal_estimation normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})


