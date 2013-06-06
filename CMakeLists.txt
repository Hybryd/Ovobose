cmake_minimum_required( VERSION 2.8 )

project( Ovobose )

find_package( OpenCV REQUIRED )
find_package( PCL 1.2 REQUIRED )
FIND_PACKAGE( Qt4 REQUIRED )


# Libraries
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
ADD_DEFINITIONS(${QT_DEFINITIONS})
INCLUDE(${QT_USE_FILE})

# Avoid depreciated declaration warnings
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

# Executables target folder
set( EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE} )


# List all common sources
file( GLOB allsrc src/*.cpp )


#SET(main_qt_src main.cpp MainWin.cpp)
#SET(main_qt_headers MainWin.h)
#QT4_WRAP_CPP(main_qt_headers_moc ${main_qt_headers})
#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_EXECUTABLE(main_qt ${main_qt_src} ${main_qt_headers_moc})
#TARGET_LINK_LIBRARIES(main_qt ${QT_LIBRARIES})
file( GLOB main_gui src/gui/*.cpp )
file( GLOB gui_src src/gui/MainWin.cpp )
file( GLOB gui_head src/gui/MainWin.h )
QT4_WRAP_CPP(gui_head_moc ${gui_head})
add_executable(gui ${main_gui} ${gui_head_moc})
target_link_libraries(gui ${QT_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# scan
file( GLOB main_scan src/scan/*.cpp )
add_executable( scan ${main_scan} ${allsrc} )
target_link_libraries( scan ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# postProcess
file(GLOB main_postProcess src/postProcess/*.cpp )
add_executable( postProcess ${main_postProcess} ${allsrc})
target_link_libraries( postProcess ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# see
file(GLOB main_see src/see/*.cpp )
add_executable( see ${main_see} ${allsrc})
target_link_libraries( see ${OpenCV_LIBS} ${PCL_LIBRARIES})

# surfaceReconstruction
file(GLOB main_surfaceReconstruction src/surfaceReconstruction/*.cpp )
add_executable( surfaceReconstruction ${main_surfaceReconstruction} ${allsrc})
target_link_libraries( surfaceReconstruction ${OpenCV_LIBS} ${PCL_LIBRARIES} )







