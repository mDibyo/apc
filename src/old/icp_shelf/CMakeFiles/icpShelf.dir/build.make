# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nm/workspace/apc/src/icp_shelf

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nm/workspace/apc/src/icp_shelf

# Include any dependencies generated for this target.
include CMakeFiles/icpShelf.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/icpShelf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/icpShelf.dir/flags.make

CMakeFiles/icpShelf.dir/icpShelf.cpp.o: CMakeFiles/icpShelf.dir/flags.make
CMakeFiles/icpShelf.dir/icpShelf.cpp.o: icpShelf.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nm/workspace/apc/src/icp_shelf/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/icpShelf.dir/icpShelf.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/icpShelf.dir/icpShelf.cpp.o -c /home/nm/workspace/apc/src/icp_shelf/icpShelf.cpp

CMakeFiles/icpShelf.dir/icpShelf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/icpShelf.dir/icpShelf.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/nm/workspace/apc/src/icp_shelf/icpShelf.cpp > CMakeFiles/icpShelf.dir/icpShelf.cpp.i

CMakeFiles/icpShelf.dir/icpShelf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/icpShelf.dir/icpShelf.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/nm/workspace/apc/src/icp_shelf/icpShelf.cpp -o CMakeFiles/icpShelf.dir/icpShelf.cpp.s

CMakeFiles/icpShelf.dir/icpShelf.cpp.o.requires:
.PHONY : CMakeFiles/icpShelf.dir/icpShelf.cpp.o.requires

CMakeFiles/icpShelf.dir/icpShelf.cpp.o.provides: CMakeFiles/icpShelf.dir/icpShelf.cpp.o.requires
	$(MAKE) -f CMakeFiles/icpShelf.dir/build.make CMakeFiles/icpShelf.dir/icpShelf.cpp.o.provides.build
.PHONY : CMakeFiles/icpShelf.dir/icpShelf.cpp.o.provides

CMakeFiles/icpShelf.dir/icpShelf.cpp.o.provides.build: CMakeFiles/icpShelf.dir/icpShelf.cpp.o

# Object files for target icpShelf
icpShelf_OBJECTS = \
"CMakeFiles/icpShelf.dir/icpShelf.cpp.o"

# External object files for target icpShelf
icpShelf_EXTERNAL_OBJECTS =

icpShelf: CMakeFiles/icpShelf.dir/icpShelf.cpp.o
icpShelf: libMyPCLUtils.a
icpShelf: /usr/lib/libboost_system-mt.so
icpShelf: /usr/lib/libboost_filesystem-mt.so
icpShelf: /usr/lib/libboost_thread-mt.so
icpShelf: /usr/lib/libboost_date_time-mt.so
icpShelf: /usr/lib/libboost_iostreams-mt.so
icpShelf: /opt/ros/groovy/lib/libpcl_common.so
icpShelf: /opt/ros/groovy/lib/libpcl_octree.so
icpShelf: /usr/lib/libOpenNI.so
icpShelf: /usr/lib/libvtkCommon.so.5.8.0
icpShelf: /usr/lib/libvtkRendering.so.5.8.0
icpShelf: /usr/lib/libvtkHybrid.so.5.8.0
icpShelf: /opt/ros/groovy/lib/libpcl_io.so
icpShelf: /opt/ros/groovy/lib/libflann_cpp_s.a
icpShelf: /opt/ros/groovy/lib/libpcl_kdtree.so
icpShelf: /opt/ros/groovy/lib/libpcl_search.so
icpShelf: /opt/ros/groovy/lib/libpcl_sample_consensus.so
icpShelf: /opt/ros/groovy/lib/libpcl_filters.so
icpShelf: /opt/ros/groovy/lib/libpcl_segmentation.so
icpShelf: /opt/ros/groovy/lib/libpcl_visualization.so
icpShelf: /opt/ros/groovy/lib/libpcl_features.so
icpShelf: /usr/lib/libqhull.so
icpShelf: /opt/ros/groovy/lib/libpcl_surface.so
icpShelf: /opt/ros/groovy/lib/libpcl_registration.so
icpShelf: /opt/ros/groovy/lib/libpcl_keypoints.so
icpShelf: /opt/ros/groovy/lib/libpcl_tracking.so
icpShelf: /usr/lib/libvtkParallel.so.5.8.0
icpShelf: /usr/lib/libvtkRendering.so.5.8.0
icpShelf: /usr/lib/libvtkGraphics.so.5.8.0
icpShelf: /usr/lib/libvtkImaging.so.5.8.0
icpShelf: /usr/lib/libvtkIO.so.5.8.0
icpShelf: /usr/lib/libvtkFiltering.so.5.8.0
icpShelf: /usr/lib/libvtkCommon.so.5.8.0
icpShelf: /usr/lib/libvtksys.so.5.8.0
icpShelf: CMakeFiles/icpShelf.dir/build.make
icpShelf: CMakeFiles/icpShelf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable icpShelf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/icpShelf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/icpShelf.dir/build: icpShelf
.PHONY : CMakeFiles/icpShelf.dir/build

CMakeFiles/icpShelf.dir/requires: CMakeFiles/icpShelf.dir/icpShelf.cpp.o.requires
.PHONY : CMakeFiles/icpShelf.dir/requires

CMakeFiles/icpShelf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/icpShelf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/icpShelf.dir/clean

CMakeFiles/icpShelf.dir/depend:
	cd /home/nm/workspace/apc/src/icp_shelf && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf/CMakeFiles/icpShelf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/icpShelf.dir/depend

