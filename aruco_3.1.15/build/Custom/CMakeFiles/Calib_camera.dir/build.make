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
include Custom/CMakeFiles/Calib_camera.dir/depend.make

# Include the progress variables for this target.
include Custom/CMakeFiles/Calib_camera.dir/progress.make

# Include the compile flags for this target's objects.
include Custom/CMakeFiles/Calib_camera.dir/flags.make

Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o: Custom/CMakeFiles/Calib_camera.dir/flags.make
Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o: ../Custom/calibration_camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o -c /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/calibration_camera.cpp

Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Calib_camera.dir/calibration_camera.cpp.i"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/calibration_camera.cpp > CMakeFiles/Calib_camera.dir/calibration_camera.cpp.i

Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Calib_camera.dir/calibration_camera.cpp.s"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom/calibration_camera.cpp -o CMakeFiles/Calib_camera.dir/calibration_camera.cpp.s

Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.requires:

.PHONY : Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.requires

Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.provides: Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.requires
	$(MAKE) -f Custom/CMakeFiles/Calib_camera.dir/build.make Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.provides.build
.PHONY : Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.provides

Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.provides.build: Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o


# Object files for target Calib_camera
Calib_camera_OBJECTS = \
"CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o"

# External object files for target Calib_camera
Calib_camera_EXTERNAL_OBJECTS =

Custom/Calib_camera: Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o
Custom/Calib_camera: Custom/CMakeFiles/Calib_camera.dir/build.make
Custom/Calib_camera: Custom/helpers/libhelpers.so
Custom/Calib_camera: src/libaruco.so.3.1.15
Custom/Calib_camera: /usr/local/lib/libopencv_calib3d.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_highgui.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_features2d.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_flann.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_videoio.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_imgproc.so.4.5.5
Custom/Calib_camera: /usr/local/lib/libopencv_core.so.4.5.5
Custom/Calib_camera: Custom/CMakeFiles/Calib_camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Calib_camera"
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Calib_camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Custom/CMakeFiles/Calib_camera.dir/build: Custom/Calib_camera

.PHONY : Custom/CMakeFiles/Calib_camera.dir/build

Custom/CMakeFiles/Calib_camera.dir/requires: Custom/CMakeFiles/Calib_camera.dir/calibration_camera.cpp.o.requires

.PHONY : Custom/CMakeFiles/Calib_camera.dir/requires

Custom/CMakeFiles/Calib_camera.dir/clean:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom && $(CMAKE_COMMAND) -P CMakeFiles/Calib_camera.dir/cmake_clean.cmake
.PHONY : Custom/CMakeFiles/Calib_camera.dir/clean

Custom/CMakeFiles/Calib_camera.dir/depend:
	cd /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15 /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/Custom /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/Custom/CMakeFiles/Calib_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Custom/CMakeFiles/Calib_camera.dir/depend

