# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /home/red1/Softwares/clion-2017.1.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/red1/Softwares/clion-2017.1.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/red1/catkin_ws/src/introlab-ros-pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug

# Include any dependencies generated for this target.
include rtaudio/tests/CMakeFiles/playraw.dir/depend.make

# Include the progress variables for this target.
include rtaudio/tests/CMakeFiles/playraw.dir/progress.make

# Include the compile flags for this target's objects.
include rtaudio/tests/CMakeFiles/playraw.dir/flags.make

rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o: rtaudio/tests/CMakeFiles/playraw.dir/flags.make
rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o: ../rtaudio/tests/playraw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o"
	cd /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/playraw.dir/playraw.cpp.o -c /home/red1/catkin_ws/src/introlab-ros-pkg/rtaudio/tests/playraw.cpp

rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/playraw.dir/playraw.cpp.i"
	cd /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/red1/catkin_ws/src/introlab-ros-pkg/rtaudio/tests/playraw.cpp > CMakeFiles/playraw.dir/playraw.cpp.i

rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/playraw.dir/playraw.cpp.s"
	cd /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/red1/catkin_ws/src/introlab-ros-pkg/rtaudio/tests/playraw.cpp -o CMakeFiles/playraw.dir/playraw.cpp.s

rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.requires:

.PHONY : rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.requires

rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.provides: rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.requires
	$(MAKE) -f rtaudio/tests/CMakeFiles/playraw.dir/build.make rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.provides.build
.PHONY : rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.provides

rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.provides.build: rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o


# Object files for target playraw
playraw_OBJECTS = \
"CMakeFiles/playraw.dir/playraw.cpp.o"

# External object files for target playraw
playraw_EXTERNAL_OBJECTS =

rtaudio/tests/playraw: rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o
rtaudio/tests/playraw: rtaudio/tests/CMakeFiles/playraw.dir/build.make
rtaudio/tests/playraw: rtaudio/librtaudio_static.a
rtaudio/tests/playraw: /usr/lib/libasound.so
rtaudio/tests/playraw: rtaudio/tests/CMakeFiles/playraw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable playraw"
	cd /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/playraw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
rtaudio/tests/CMakeFiles/playraw.dir/build: rtaudio/tests/playraw

.PHONY : rtaudio/tests/CMakeFiles/playraw.dir/build

rtaudio/tests/CMakeFiles/playraw.dir/requires: rtaudio/tests/CMakeFiles/playraw.dir/playraw.cpp.o.requires

.PHONY : rtaudio/tests/CMakeFiles/playraw.dir/requires

rtaudio/tests/CMakeFiles/playraw.dir/clean:
	cd /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests && $(CMAKE_COMMAND) -P CMakeFiles/playraw.dir/cmake_clean.cmake
.PHONY : rtaudio/tests/CMakeFiles/playraw.dir/clean

rtaudio/tests/CMakeFiles/playraw.dir/depend:
	cd /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/red1/catkin_ws/src/introlab-ros-pkg /home/red1/catkin_ws/src/introlab-ros-pkg/rtaudio/tests /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests /home/red1/catkin_ws/src/introlab-ros-pkg/cmake-build-debug/rtaudio/tests/CMakeFiles/playraw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : rtaudio/tests/CMakeFiles/playraw.dir/depend

