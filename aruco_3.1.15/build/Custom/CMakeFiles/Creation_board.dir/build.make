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
include Custom/CMakeFiles/Creation_board.dir/depend.make

# Include the progress variables for this target.
include Custom/CMakeFiles/Creation_board.dir/progress.make

# Include the compile flags for this target's objects.
include Custom/CMakeFiles/Creation_board.dir/flags.make

Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o: Custom/CMakeFiles/Creation_board.dir/flags.make
Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o: ../Custom/create_board.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Creation_board.dir/create_board.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/create_board.cpp

Custom/CMakeFiles/Creation_board.dir/create_board.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Creation_board.dir/create_board.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/create_board.cpp > CMakeFiles/Creation_board.dir/create_board.cpp.i

Custom/CMakeFiles/Creation_board.dir/create_board.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Creation_board.dir/create_board.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/create_board.cpp -o CMakeFiles/Creation_board.dir/create_board.cpp.s

Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.requires:

.PHONY : Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.requires

Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.provides: Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.requires
	$(MAKE) -f Custom/CMakeFiles/Creation_board.dir/build.make Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.provides.build
.PHONY : Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.provides

Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.provides.build: Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o


# Object files for target Creation_board
Creation_board_OBJECTS = \
"CMakeFiles/Creation_board.dir/create_board.cpp.o"

# External object files for target Creation_board
Creation_board_EXTERNAL_OBJECTS =

Custom/Creation_board: Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o
Custom/Creation_board: Custom/CMakeFiles/Creation_board.dir/build.make
Custom/Creation_board: Custom/aruco_lib/libaruco_lib.so
Custom/Creation_board: /usr/local/lib/libopencv_calib3d.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_highgui.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_features2d.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_flann.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_videoio.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_imgproc.so.4.5.5
Custom/Creation_board: /usr/local/lib/libopencv_core.so.4.5.5
Custom/Creation_board: Custom/CMakeFiles/Creation_board.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Creation_board"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Creation_board.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Custom/CMakeFiles/Creation_board.dir/build: Custom/Creation_board

.PHONY : Custom/CMakeFiles/Creation_board.dir/build

Custom/CMakeFiles/Creation_board.dir/requires: Custom/CMakeFiles/Creation_board.dir/create_board.cpp.o.requires

.PHONY : Custom/CMakeFiles/Creation_board.dir/requires

Custom/CMakeFiles/Creation_board.dir/clean:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && $(CMAKE_COMMAND) -P CMakeFiles/Creation_board.dir/cmake_clean.cmake
.PHONY : Custom/CMakeFiles/Creation_board.dir/clean

Custom/CMakeFiles/Creation_board.dir/depend:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15 /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/CMakeFiles/Creation_board.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Custom/CMakeFiles/Creation_board.dir/depend
