cmake_minimum_required(VERSION 3.0.0)

project(Aruco_Positionning VERSION 1.0.0)

add_subdirectory(helpers)

FIND_PACKAGE( OpenCV REQUIRED )                              # 环境变量中寻找OpenCV的库的位置  
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )


add_executable(Aruco_Markers test.cpp)


target_link_libraries(Aruco_Markers PRIVATE helpers opencv_calib3d opencv_highgui)
