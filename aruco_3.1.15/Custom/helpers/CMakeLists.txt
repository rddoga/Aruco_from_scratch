#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)    #for the aruco 3 files in src

add_library(helpers
#src/calibrator.cpp
src/i2c_helper.cpp
src/utils.cpp
src/v4l2_helper.cpp)


target_include_directories(helpers PUBLIC include)
