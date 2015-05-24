cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (segmentShelf segmentShelf.cpp)
add_executable (transformCloud transformCloud.cpp)

add_library(MyPCLUtils STATIC utils.cpp)
set_target_properties(MyPCLUtils PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries (segmentShelf MyPCLUtils ${PCL_LIBRARIES})
target_link_libraries (transformCloud MyPCLUtils ${PCL_LIBRARIES})
