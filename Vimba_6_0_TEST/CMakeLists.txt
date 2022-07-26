cmake_minimum_required(VERSION 3.0.0)

project(Test_camera_catch VERSION 1.0.0)


FIND_PACKAGE( OpenCV REQUIRED )                              # 环境变量中寻找OpenCV的库的位置  
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} ) #For including openCV

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 REQUIRED  QtCore  QtGui  QtDesigner) #For finding Qt4




#add_subdirectory(Helpers_Cam)

add_executable(NewCam Test_cam.cpp)

#For finding the library path
find_library(VimbaCPP_lib VimbaCPP  HINTS  ${CMAKE_SOURCE_DIR}/Vimba_all/VimbaCPP/DynamicLib/arm_64bit ) #Looking for the .so file
find_library(VimbaC_lib VimbaC  HINTS  ${CMAKE_SOURCE_DIR}/Vimba_all/VimbaCPP/DynamicLib/arm_64bit ) #Looking for the .so file

message("VimbCPP lib : " ${VimbaCPP_lib})
message("VimbC lib : " ${VimbaC_lib})

target_include_directories(NewCam  PRIVATE  ${CMAKE_SOURCE_DIR}/Vimba_all )

#For linking the found libraries
target_link_libraries(NewCam  PRIVATE       ${VimbaC_lib}  ${VimbaCPP_lib} opencv_highgui  Qt4::QtCore) 
