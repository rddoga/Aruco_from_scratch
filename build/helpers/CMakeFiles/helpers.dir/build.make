# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rddoga/Desktop/Aruco_from_scratch

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rddoga/Desktop/Aruco_from_scratch/build

# Include any dependencies generated for this target.
include helpers/CMakeFiles/helpers.dir/depend.make

# Include the progress variables for this target.
include helpers/CMakeFiles/helpers.dir/progress.make

# Include the compile flags for this target's objects.
include helpers/CMakeFiles/helpers.dir/flags.make

helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o: helpers/CMakeFiles/helpers.dir/flags.make
helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o: ../helpers/src/i2c_helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helpers.dir/src/i2c_helper.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/i2c_helper.cpp

helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helpers.dir/src/i2c_helper.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/i2c_helper.cpp > CMakeFiles/helpers.dir/src/i2c_helper.cpp.i

helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helpers.dir/src/i2c_helper.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/i2c_helper.cpp -o CMakeFiles/helpers.dir/src/i2c_helper.cpp.s

helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.requires:

.PHONY : helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.requires

helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.provides: helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.requires
	$(MAKE) -f helpers/CMakeFiles/helpers.dir/build.make helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.provides.build
.PHONY : helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.provides

helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.provides.build: helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o


helpers/CMakeFiles/helpers.dir/src/utils.cpp.o: helpers/CMakeFiles/helpers.dir/flags.make
helpers/CMakeFiles/helpers.dir/src/utils.cpp.o: ../helpers/src/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object helpers/CMakeFiles/helpers.dir/src/utils.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helpers.dir/src/utils.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/utils.cpp

helpers/CMakeFiles/helpers.dir/src/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helpers.dir/src/utils.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/utils.cpp > CMakeFiles/helpers.dir/src/utils.cpp.i

helpers/CMakeFiles/helpers.dir/src/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helpers.dir/src/utils.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/utils.cpp -o CMakeFiles/helpers.dir/src/utils.cpp.s

helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.requires:

.PHONY : helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.requires

helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.provides: helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.requires
	$(MAKE) -f helpers/CMakeFiles/helpers.dir/build.make helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.provides.build
.PHONY : helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.provides

helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.provides.build: helpers/CMakeFiles/helpers.dir/src/utils.cpp.o


helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o: helpers/CMakeFiles/helpers.dir/flags.make
helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o: ../helpers/src/v4l2_helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/v4l2_helper.cpp

helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helpers.dir/src/v4l2_helper.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/v4l2_helper.cpp > CMakeFiles/helpers.dir/src/v4l2_helper.cpp.i

helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helpers.dir/src/v4l2_helper.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/helpers/src/v4l2_helper.cpp -o CMakeFiles/helpers.dir/src/v4l2_helper.cpp.s

helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.requires:

.PHONY : helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.requires

helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.provides: helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.requires
	$(MAKE) -f helpers/CMakeFiles/helpers.dir/build.make helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.provides.build
.PHONY : helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.provides

helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.provides.build: helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o


# Object files for target helpers
helpers_OBJECTS = \
"CMakeFiles/helpers.dir/src/i2c_helper.cpp.o" \
"CMakeFiles/helpers.dir/src/utils.cpp.o" \
"CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o"

# External object files for target helpers
helpers_EXTERNAL_OBJECTS =

helpers/libhelpers.a: helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o
helpers/libhelpers.a: helpers/CMakeFiles/helpers.dir/src/utils.cpp.o
helpers/libhelpers.a: helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o
helpers/libhelpers.a: helpers/CMakeFiles/helpers.dir/build.make
helpers/libhelpers.a: helpers/CMakeFiles/helpers.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libhelpers.a"
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && $(CMAKE_COMMAND) -P CMakeFiles/helpers.dir/cmake_clean_target.cmake
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/helpers.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
helpers/CMakeFiles/helpers.dir/build: helpers/libhelpers.a

.PHONY : helpers/CMakeFiles/helpers.dir/build

helpers/CMakeFiles/helpers.dir/requires: helpers/CMakeFiles/helpers.dir/src/i2c_helper.cpp.o.requires
helpers/CMakeFiles/helpers.dir/requires: helpers/CMakeFiles/helpers.dir/src/utils.cpp.o.requires
helpers/CMakeFiles/helpers.dir/requires: helpers/CMakeFiles/helpers.dir/src/v4l2_helper.cpp.o.requires

.PHONY : helpers/CMakeFiles/helpers.dir/requires

helpers/CMakeFiles/helpers.dir/clean:
	cd /home/rddoga/Desktop/Aruco_from_scratch/build/helpers && $(CMAKE_COMMAND) -P CMakeFiles/helpers.dir/cmake_clean.cmake
.PHONY : helpers/CMakeFiles/helpers.dir/clean

helpers/CMakeFiles/helpers.dir/depend:
	cd /home/rddoga/Desktop/Aruco_from_scratch/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rddoga/Desktop/Aruco_from_scratch /home/rddoga/Desktop/Aruco_from_scratch/helpers /home/rddoga/Desktop/Aruco_from_scratch/build /home/rddoga/Desktop/Aruco_from_scratch/build/helpers /home/rddoga/Desktop/Aruco_from_scratch/build/helpers/CMakeFiles/helpers.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : helpers/CMakeFiles/helpers.dir/depend
