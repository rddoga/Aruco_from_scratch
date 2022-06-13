
add_library(Alvium_Camera_Helpers
src/Alvium_Camera.cpp
)

target_include_directories(Alvium_Camera_Helpers  PUBLIC  ${CMAKE_SOURCE_DIR}/Custom/Vimba_6_0   include)
