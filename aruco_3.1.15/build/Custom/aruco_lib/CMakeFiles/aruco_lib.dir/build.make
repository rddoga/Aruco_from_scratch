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
include Custom/aruco_lib/CMakeFiles/aruco_lib.dir/depend.make

# Include the progress variables for this target.
include Custom/aruco_lib/CMakeFiles/aruco_lib.dir/progress.make

# Include the compile flags for this target's objects.
include Custom/aruco_lib/CMakeFiles/aruco_lib.dir/flags.make

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/flags.make
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o: ../Custom/aruco_lib/src/apriltag_quad_thresh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/apriltag_quad_thresh.cpp

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/apriltag_quad_thresh.cpp > CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.i

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/apriltag_quad_thresh.cpp -o CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.s

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.requires:

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.requires

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.provides: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.requires
	$(MAKE) -f Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build.make Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.provides.build
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.provides

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.provides.build: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o


Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/flags.make
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o: ../Custom/aruco_lib/src/aruco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_lib.dir/src/aruco.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/aruco.cpp

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_lib.dir/src/aruco.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/aruco.cpp > CMakeFiles/aruco_lib.dir/src/aruco.cpp.i

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_lib.dir/src/aruco.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/aruco.cpp -o CMakeFiles/aruco_lib.dir/src/aruco.cpp.s

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.requires:

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.requires

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.provides: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.requires
	$(MAKE) -f Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build.make Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.provides.build
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.provides

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.provides.build: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o


Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/flags.make
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o: ../Custom/aruco_lib/src/charuco.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_lib.dir/src/charuco.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/charuco.cpp

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_lib.dir/src/charuco.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/charuco.cpp > CMakeFiles/aruco_lib.dir/src/charuco.cpp.i

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_lib.dir/src/charuco.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/charuco.cpp -o CMakeFiles/aruco_lib.dir/src/charuco.cpp.s

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.requires:

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.requires

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.provides: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.requires
	$(MAKE) -f Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build.make Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.provides.build
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.provides

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.provides.build: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o


Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/flags.make
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o: ../Custom/aruco_lib/src/dictionary.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/dictionary.cpp

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_lib.dir/src/dictionary.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/dictionary.cpp > CMakeFiles/aruco_lib.dir/src/dictionary.cpp.i

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_lib.dir/src/dictionary.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/dictionary.cpp -o CMakeFiles/aruco_lib.dir/src/dictionary.cpp.s

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.requires:

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.requires

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.provides: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.requires
	$(MAKE) -f Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build.make Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.provides.build
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.provides

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.provides.build: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o


Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/flags.make
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o: ../Custom/aruco_lib/src/zmaxheap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/zmaxheap.cpp

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/zmaxheap.cpp > CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.i

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib/src/zmaxheap.cpp -o CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.s

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.requires:

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.requires

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.provides: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.requires
	$(MAKE) -f Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build.make Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.provides.build
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.provides

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.provides.build: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o


# Object files for target aruco_lib
aruco_lib_OBJECTS = \
"CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o" \
"CMakeFiles/aruco_lib.dir/src/aruco.cpp.o" \
"CMakeFiles/aruco_lib.dir/src/charuco.cpp.o" \
"CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o" \
"CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o"

# External object files for target aruco_lib
aruco_lib_EXTERNAL_OBJECTS =

Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o
Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o
Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o
Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o
Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o
Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build.make
Custom/aruco_lib/libaruco_lib.so: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libaruco_lib.so"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build: Custom/aruco_lib/libaruco_lib.so

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/build

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/requires: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/apriltag_quad_thresh.cpp.o.requires
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/requires: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/aruco.cpp.o.requires
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/requires: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/charuco.cpp.o.requires
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/requires: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/dictionary.cpp.o.requires
Custom/aruco_lib/CMakeFiles/aruco_lib.dir/requires: Custom/aruco_lib/CMakeFiles/aruco_lib.dir/src/zmaxheap.cpp.o.requires

.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/requires

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/clean:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib && $(CMAKE_COMMAND) -P CMakeFiles/aruco_lib.dir/cmake_clean.cmake
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/clean

Custom/aruco_lib/CMakeFiles/aruco_lib.dir/depend:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15 /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/aruco_lib /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/aruco_lib/CMakeFiles/aruco_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Custom/aruco_lib/CMakeFiles/aruco_lib.dir/depend

