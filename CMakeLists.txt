cmake_minimum_required(VERSION 2.8)
project( Ovobose )
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")


add_executable( main main.cpp Calibration.h Calibration.cpp PostProcessor.h PostProcessor.cpp Scan.h Scan.cpp)
target_link_libraries( main ${OpenCV_LIBS} )

add_executable( see see.cpp )
target_link_libraries( see ${OpenCV_LIBS} )

