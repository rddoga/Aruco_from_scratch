
cmake_minimum_required(VERSION 3.0.0)

project(Aruco_Positionning VERSION 1.0.0)

add_subdirectory(helpers)
add_subdirectory(aruco_lib)

FIND_PACKAGE( OpenCV REQUIRED )                              # 环境变量中寻找OpenCV的库的位置  
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

add_executable(Aruco_Markers test_aruco2_singleMarkers.cpp)
add_executable(Aruco_Board test_aruco2_Board.cpp)
add_executable(Aruco3_Markers test_aruco3.cpp)
add_executable(Creation_markers create_marker.cpp) # Pour créer et imprimer les markers
add_executable(Creation_board create_board.cpp) # Pour créer une board de markers
add_executable(Calib_camera calibration_camera.cpp)

target_link_libraries(Aruco_Markers PRIVATE         helpers  aruco_lib  opencv_calib3d  opencv_highgui)
target_link_libraries(Aruco_Board PRIVATE           helpers  aruco_lib  opencv_calib3d  opencv_highgui)
target_link_libraries(Aruco3_Markers PRIVATE        helpers  aruco  opencv_calib3d  opencv_highgui)
target_link_libraries(Creation_markers PRIVATE      aruco_lib  opencv_calib3d  opencv_highgui)
target_link_libraries(Creation_board PRIVATE        aruco_lib  opencv_calib3d  opencv_highgui)
target_link_libraries(Calib_camera PRIVATE          helpers  aruco  opencv_calib3d  opencv_highgui)

#install(TARGETS Aruco_Markers Aruco3_Markers Creation_markers Calib_camera RUNTIME DESTINATION bin)