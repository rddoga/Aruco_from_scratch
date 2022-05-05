# Install script for directory: /home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xmainx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1.15"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/src/libaruco.so.3.1.15"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/src/libaruco.so.3.1"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/build/src/libaruco.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1.15"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.3.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/aruco_cvversioning.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/cameraparameters.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/dictionary_based.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/ippe.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/markerdetector_impl.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/markermap.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/timers.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/aruco_export.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/cvdrawingutils.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/dictionary.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/levmarq.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/marker.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/picoflann.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/aruco.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/debug.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/markerdetector.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/markerlabeler.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/posetracker.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/fractaldetector.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco/fractallabelers" TYPE FILE FILES
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/fractallabelers/fractalposetracker.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/fractallabelers/fractalmarkerset.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/fractallabelers/fractalmarker.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/fractallabelers/fractallabeler.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco/dcf" TYPE FILE FILES
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/dcf/dcfmarkermaptracker.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/dcf/dcfmarkertracker.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/dcf/dcf_utils.h"
    "/home/rddoga/Desktop/Aruco_from_scratch/aruco_3.1.15/src/dcf/trackerimpl.h"
    )
endif()

