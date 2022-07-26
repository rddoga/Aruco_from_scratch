INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/Vimba_all) 


add_library(Helpers_Cam
src/ApiController.cpp
#src/AsynchronousGrab.cpp
src/CameraObserver.cpp
src/FrameObserver.cpp
#src/main.cpp
)

target_include_directories(Helpers_Cam PUBLIC include)

target_link_libraries(Helpers_Cam PUBLIC Qt4::QtCore Qt4::QtGui Qt4::QtDesigner) #For accessing Qt include files
