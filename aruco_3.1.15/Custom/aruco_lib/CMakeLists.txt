add_library(aruco_lib
src/apriltag_quad_thresh.cpp
src/aruco.cpp
src/charuco.cpp
src/dictionary.cpp
src/zmaxheap.cpp)
#src/create_marker.cpp)


target_include_directories(aruco_lib PUBLIC include)
