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
include CMakeFiles/corner3D.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/corner3D.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/corner3D.dir/flags.make

CMakeFiles/corner3D.dir/corner3D.cpp.o: CMakeFiles/corner3D.dir/flags.make
CMakeFiles/corner3D.dir/corner3D.cpp.o: corner3D.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nm/workspace/apc/src/icp_shelf/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/corner3D.dir/corner3D.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/corner3D.dir/corner3D.cpp.o -c /home/nm/workspace/apc/src/icp_shelf/corner3D.cpp

CMakeFiles/corner3D.dir/corner3D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/corner3D.dir/corner3D.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/nm/workspace/apc/src/icp_shelf/corner3D.cpp > CMakeFiles/corner3D.dir/corner3D.cpp.i

CMakeFiles/corner3D.dir/corner3D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/corner3D.dir/corner3D.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/nm/workspace/apc/src/icp_shelf/corner3D.cpp -o CMakeFiles/corner3D.dir/corner3D.cpp.s

CMakeFiles/corner3D.dir/corner3D.cpp.o.requires:
.PHONY : CMakeFiles/corner3D.dir/corner3D.cpp.o.requires

CMakeFiles/corner3D.dir/corner3D.cpp.o.provides: CMakeFiles/corner3D.dir/corner3D.cpp.o.requires
	$(MAKE) -f CMakeFiles/corner3D.dir/build.make CMakeFiles/corner3D.dir/corner3D.cpp.o.provides.build
.PHONY : CMakeFiles/corner3D.dir/corner3D.cpp.o.provides

CMakeFiles/corner3D.dir/corner3D.cpp.o.provides.build: CMakeFiles/corner3D.dir/corner3D.cpp.o

# Object files for target corner3D
corner3D_OBJECTS = \
"CMakeFiles/corner3D.dir/corner3D.cpp.o"

# External object files for target corner3D
corner3D_EXTERNAL_OBJECTS =

corner3D: CMakeFiles/corner3D.dir/corner3D.cpp.o
corner3D: libMyPCLUtils.a
corner3D: /usr/lib/libboost_system-mt.so
corner3D: /usr/lib/libboost_filesystem-mt.so
corner3D: /usr/lib/libboost_thread-mt.so
corner3D: /usr/lib/libboost_date_time-mt.so
corner3D: /usr/lib/libboost_iostreams-mt.so
corner3D: /opt/ros/groovy/lib/libpcl_common.so
corner3D: /opt/ros/groovy/lib/libpcl_octree.so
corner3D: /usr/lib/libOpenNI.so
corner3D: /usr/lib/libvtkCommon.so.5.8.0
corner3D: /usr/lib/libvtkRendering.so.5.8.0
corner3D: /usr/lib/libvtkHybrid.so.5.8.0
corner3D: /opt/ros/groovy/lib/libpcl_io.so
corner3D: /opt/ros/groovy/lib/libflann_cpp_s.a
corner3D: /opt/ros/groovy/lib/libpcl_kdtree.so
corner3D: /opt/ros/groovy/lib/libpcl_search.so
corner3D: /opt/ros/groovy/lib/libpcl_sample_consensus.so
corner3D: /opt/ros/groovy/lib/libpcl_filters.so
corner3D: /opt/ros/groovy/lib/libpcl_segmentation.so
corner3D: /opt/ros/groovy/lib/libpcl_visualization.so
corner3D: /opt/ros/groovy/lib/libpcl_features.so
corner3D: /usr/lib/libqhull.so
corner3D: /opt/ros/groovy/lib/libpcl_surface.so
corner3D: /opt/ros/groovy/lib/libpcl_registration.so
corner3D: /opt/ros/groovy/lib/libpcl_keypoints.so
corner3D: /opt/ros/groovy/lib/libpcl_tracking.so
corner3D: /usr/lib/libvtkParallel.so.5.8.0
corner3D: /usr/lib/libvtkRendering.so.5.8.0
corner3D: /usr/lib/libvtkGraphics.so.5.8.0
corner3D: /usr/lib/libvtkImaging.so.5.8.0
corner3D: /usr/lib/libvtkIO.so.5.8.0
corner3D: /usr/lib/libvtkFiltering.so.5.8.0
corner3D: /usr/lib/libvtkCommon.so.5.8.0
corner3D: /usr/lib/libvtksys.so.5.8.0
corner3D: CMakeFiles/corner3D.dir/build.make
corner3D: CMakeFiles/corner3D.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable corner3D"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/corner3D.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/corner3D.dir/build: corner3D
.PHONY : CMakeFiles/corner3D.dir/build

CMakeFiles/corner3D.dir/requires: CMakeFiles/corner3D.dir/corner3D.cpp.o.requires
.PHONY : CMakeFiles/corner3D.dir/requires

CMakeFiles/corner3D.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/corner3D.dir/cmake_clean.cmake
.PHONY : CMakeFiles/corner3D.dir/clean

CMakeFiles/corner3D.dir/depend:
	cd /home/nm/workspace/apc/src/icp_shelf && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf /home/nm/workspace/apc/src/icp_shelf/CMakeFiles/corner3D.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/corner3D.dir/depend

