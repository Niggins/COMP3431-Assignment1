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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alex/comp3431/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alex/comp3431/build

# Include any dependencies generated for this target.
include assign1_2013/CMakeFiles/safety_first.dir/depend.make

# Include the progress variables for this target.
include assign1_2013/CMakeFiles/safety_first.dir/progress.make

# Include the compile flags for this target's objects.
include assign1_2013/CMakeFiles/safety_first.dir/flags.make

assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o: assign1_2013/CMakeFiles/safety_first.dir/flags.make
assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o: /home/alex/comp3431/src/assign1_2013/src/nodes/safety_first.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/alex/comp3431/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o"
	cd /home/alex/comp3431/build/assign1_2013 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o -c /home/alex/comp3431/src/assign1_2013/src/nodes/safety_first.cpp

assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.i"
	cd /home/alex/comp3431/build/assign1_2013 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/alex/comp3431/src/assign1_2013/src/nodes/safety_first.cpp > CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.i

assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.s"
	cd /home/alex/comp3431/build/assign1_2013 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/alex/comp3431/src/assign1_2013/src/nodes/safety_first.cpp -o CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.s

assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.requires:
.PHONY : assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.requires

assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.provides: assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.requires
	$(MAKE) -f assign1_2013/CMakeFiles/safety_first.dir/build.make assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.provides.build
.PHONY : assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.provides

assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.provides.build: assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o

# Object files for target safety_first
safety_first_OBJECTS = \
"CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o"

# External object files for target safety_first
safety_first_EXTERNAL_OBJECTS =

/home/alex/comp3431/devel/lib/assign1_2013/safety_first: assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: assign1_2013/CMakeFiles/safety_first.dir/build.make
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libcv_bridge.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_calib3d.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_contrib.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_core.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_features2d.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_flann.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_gpu.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_highgui.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_imgproc.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_legacy.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_ml.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_nonfree.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_objdetect.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_photo.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_stitching.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_superres.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_ts.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_video.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libopencv_videostab.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/librosconsole.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libboost_regex-mt.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libboost_thread-mt.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/liblog4cxx.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libcpp_common.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/librostime.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libboost_date_time-mt.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libboost_system-mt.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libroscpp_serialization.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libroscpp.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libboost_signals-mt.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libboost_filesystem-mt.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libxmlrpcpp.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libtf.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libmessage_filters.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libimage_transport.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libtinyxml.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libclass_loader.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/libPocoFoundation.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /usr/lib/x86_64-linux-gnu/libdl.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libconsole_bridge.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: /opt/ros/groovy/lib/libroslib.so
/home/alex/comp3431/devel/lib/assign1_2013/safety_first: assign1_2013/CMakeFiles/safety_first.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/alex/comp3431/devel/lib/assign1_2013/safety_first"
	cd /home/alex/comp3431/build/assign1_2013 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/safety_first.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
assign1_2013/CMakeFiles/safety_first.dir/build: /home/alex/comp3431/devel/lib/assign1_2013/safety_first
.PHONY : assign1_2013/CMakeFiles/safety_first.dir/build

assign1_2013/CMakeFiles/safety_first.dir/requires: assign1_2013/CMakeFiles/safety_first.dir/src/nodes/safety_first.cpp.o.requires
.PHONY : assign1_2013/CMakeFiles/safety_first.dir/requires

assign1_2013/CMakeFiles/safety_first.dir/clean:
	cd /home/alex/comp3431/build/assign1_2013 && $(CMAKE_COMMAND) -P CMakeFiles/safety_first.dir/cmake_clean.cmake
.PHONY : assign1_2013/CMakeFiles/safety_first.dir/clean

assign1_2013/CMakeFiles/safety_first.dir/depend:
	cd /home/alex/comp3431/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/comp3431/src /home/alex/comp3431/src/assign1_2013 /home/alex/comp3431/build /home/alex/comp3431/build/assign1_2013 /home/alex/comp3431/build/assign1_2013/CMakeFiles/safety_first.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : assign1_2013/CMakeFiles/safety_first.dir/depend

