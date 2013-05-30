cmake_minimum_required( VERSION 2.8 )

project( Ovobose )

find_package( OpenCV REQUIRED )
find_package( PCL 1.2 REQUIRED )


# Libraries
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Avoid depreciated declaration warnings
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

# Executables target folder
set( EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE} )


# List all common sources
file( GLOB allsrc src/*.cpp )

# scan
file( GLOB main_scan src/scan/* )
add_executable( scan ${main_scan} ${allsrc} )
target_link_libraries( scan ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# postProcess
file(GLOB main_postProcess src/postProcess/* )
add_executable( postProcess ${main_postProcess} ${allsrc})
target_link_libraries( postProcess ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# see
file(GLOB main_see src/see/* )
add_executable( see ${main_see} ${allsrc})
target_link_libraries( see ${OpenCV_LIBS} ${PCL_LIBRARIES})

# surfaceReconstruction
file(GLOB main_surfaceReconstruction src/surfaceReconstruction/* )
add_executable( surfaceReconstruction ${main_surfaceReconstruction} ${allsrc})
target_link_libraries( surfaceReconstruction ${OpenCV_LIBS} ${PCL_LIBRARIES} )


