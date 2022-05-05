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
CMAKE_SOURCE_DIR = /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_batch_processing.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_batch_processing.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_batch_processing.dir/flags.make

utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o: utils/CMakeFiles/aruco_batch_processing.dir/flags.make
utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o: ../utils/aruco_batch_processing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/utils/aruco_batch_processing.cpp

utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/utils/aruco_batch_processing.cpp > CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.i

utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/utils/aruco_batch_processing.cpp -o CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.s

utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.requires:

.PHONY : utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.requires

utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.provides: utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_batch_processing.dir/build.make utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.provides

utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.provides.build: utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o


# Object files for target aruco_batch_processing
aruco_batch_processing_OBJECTS = \
"CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o"

# External object files for target aruco_batch_processing
aruco_batch_processing_EXTERNAL_OBJECTS =

utils/aruco_batch_processing: utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o
utils/aruco_batch_processing: utils/CMakeFiles/aruco_batch_processing.dir/build.make
utils/aruco_batch_processing: src/libaruco.so.3.1.15
utils/aruco_batch_processing: /usr/local/lib/libopencv_calib3d.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_highgui.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_features2d.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_flann.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_videoio.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_imgproc.so.4.5.5
utils/aruco_batch_processing: /usr/local/lib/libopencv_core.so.4.5.5
utils/aruco_batch_processing: utils/CMakeFiles/aruco_batch_processing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable aruco_batch_processing"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_batch_processing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_batch_processing.dir/build: utils/aruco_batch_processing

.PHONY : utils/CMakeFiles/aruco_batch_processing.dir/build

utils/CMakeFiles/aruco_batch_processing.dir/requires: utils/CMakeFiles/aruco_batch_processing.dir/aruco_batch_processing.cpp.o.requires

.PHONY : utils/CMakeFiles/aruco_batch_processing.dir/requires

utils/CMakeFiles/aruco_batch_processing.dir/clean:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_batch_processing.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_batch_processing.dir/clean

utils/CMakeFiles/aruco_batch_processing.dir/depend:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15 /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/utils /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/utils/CMakeFiles/aruco_batch_processing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_batch_processing.dir/depend

