cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (icpShelf icpShelf.cpp)
#target_link_libraries (icpShelf ${PCL_LIBRARIES})



#add_executable (template_alignment template_alignment.cpp)
#target_link_libraries (template_alignment ${PCL_LIBRARIES})

add_executable (alignBox alignBox.cpp)
#add_executable (template_alignment template_alignment.cpp)
add_library(MyPCLUtils STATIC utils.cpp)
set_target_properties(MyPCLUtils PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries (alignBox MyPCLUtils ${PCL_LIBRARIES})
#target_link_libraries (template_alignment MyPCLUtils ${PCL_LIBRARIES})
