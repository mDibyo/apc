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
include CMakeFiles/ply2pcd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ply2pcd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ply2pcd.dir/flags.make

CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o: CMakeFiles/ply2pcd.dir/flags.make
CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o: ply2pcd.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nm/workspace/apc/src/icp_shelf/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o -c /home/nm/workspace/apc/src/icp_shelf/ply2pcd.cpp

CMakeFiles/ply2pcd.dir/ply2pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ply2pcd.dir/ply2pcd.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/nm/workspace/apc/src/icp_shelf/ply2pcd.cpp > CMakeFiles/ply2pcd.dir/ply2pcd.cpp.i

CMakeFiles/ply2pcd.dir/ply2pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ply2pcd.dir/ply2pcd.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/nm/workspace/apc/src/icp_shelf/ply2pcd.cpp -o CMakeFiles/ply2pcd.dir/ply2pcd.cpp.s

CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.requires:
.PHONY : CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.requires

CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.provides: CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.requires
	$(MAKE) -f CMakeFiles/ply2pcd.dir/build.make CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.provides.build
.PHONY : CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.provides

CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.provides.build: CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o

# Object files for target ply2pcd
ply2pcd_OBJECTS = \
"CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o"

# External object files for target ply2pcd
ply2pcd_EXTERNAL_OBJECTS =

ply2pcd: CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o
ply2pcd: /usr/lib/libboost_system-mt.so
ply2pcd: /usr/lib/libboost_filesystem-mt.so
ply2pcd: /usr/lib/libboost_thread-mt.so
ply2pcd: /usr/lib/libboost_date_time-mt.so
ply2pcd: /usr/lib/libboost_iostreams-mt.so
ply2pcd: /opt/ros/groovy/lib/libpcl_common.so
ply2pcd: /opt/ros/groovy/lib/libpcl_octree.so
ply2pcd: /usr/lib/libOpenNI.so
ply2pcd: /usr/lib/libvtkCommon.so.5.8.0
ply2pcd: /usr/lib/libvtkRendering.so.5.8.0
ply2pcd: /usr/lib/libvtkHybrid.so.5.8.0
ply2pcd: /opt/ros/groovy/lib/libpcl_io.so
ply2pcd: /opt/ros/groovy/lib/libflann_cpp_s.a
ply2pcd: /opt/ros/groovy/lib/libpcl_kdtree.so
ply2pcd: /opt/ros/groovy/lib/libpcl_search.so
ply2pcd: /opt/ros/groovy/lib/libpcl_sample_consensus.so
ply2pcd: /opt/ros/groovy/lib/libpcl_filters.so
ply2pcd: /opt/ros/groovy/lib/libpcl_segmentation.so
ply2pcd: /opt/ros/groovy/lib/libpcl_visualization.so
ply2pcd: /opt/ros/groovy/lib/libpcl_features.so
ply2pcd: /usr/lib/libqhull.so
ply2pcd: /opt/ros/groovy/lib/libpcl_surface.so
ply2pcd: /opt/ros/groovy/lib/libpcl_registration.so
ply2pcd: /opt/ros/groovy/lib/libpcl_keypoints.so
ply2pcd: /opt/ros/groovy/lib/libpcl_tracking.so
ply2pcd: /usr/lib/libvtkParallel.so.5.8.0
ply2pcd: /usr/lib/libvtkRendering.so.5.8.0
ply2pcd: /usr/lib/libvtkGraphics.so.5.8.0
ply2pcd: /usr/lib/libvtkImaging.so.5.8.0
ply2pcd: /usr/lib/libvtkIO.so.5.8.0
ply2pcd: /usr/lib/libvtkFiltering.so.5.8.0
ply2pcd: /usr/lib/libvtkCommon.so.5.8.0
ply2pcd: /usr/lib/libvtksys.so.5.8.0
ply2pcd: CMakeFiles/ply2pcd.dir/build.make
ply2pcd: CMakeFiles/ply2pcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ply2pcd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ply2pcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ply2pcd.dir/build: ply2pcd
.PHONY : CMakeFiles/ply2pcd.dir/build

CMakeFiles/ply2pcd.dir/requires: CMakeFiles/ply2pcd.dir/ply2pcd.cpp.o.requires
.PHONY : CMakeFiles/ply2pcd.dir/requires

CMakeFiles/ply2pcd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ply2pcd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ply2pcd.dir/clean

CMakeFiles/ply2pcd.dir/depend:
	cd /home/nm/workspace/apc/src/icp_shelf && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf/CMakeFiles/ply2pcd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ply2pcd.dir/depend

