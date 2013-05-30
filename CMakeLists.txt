cmake_minimum_required( VERSION 2.8 )

project( Ovobose )

find_package( OpenCV REQUIRED )
find_package( PCL 1.2 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")


add_executable( ./bin/scan ./src/main_scan.cpp ./src/Calibration.h ./src/Calibration.cpp ./src/PostProcessor.h ./src/PostProcessor.cpp ./src/Scan.h ./src/Scan.cpp)
target_link_libraries( ./bin/scan ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( ./bin/see ./src/main_see.cpp )
target_link_libraries( ./bin/see ${OpenCV_LIBS} )

add_executable( ./bin/surfaceReconstruction ./src/main_surfaceReconstruction.cpp )
target_link_libraries( ./bin/surfaceReconstruction ${PCL_LIBRARIES} )
