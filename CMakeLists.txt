

cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true )
 
if ( COMMAND cmake_policy )

  cmake_policy( SET CMP0003 NEW )  

endif()

project( Ovobose )


find_package( OpenCV REQUIRED )
#find_package( PCL 1.2 REQUIRED )
#include_directories( ${PCL_INCLUDE_DIRS} )
#link_directories( ${PCL_LIBRARY_DIRS} )
#add_definitions( ${PCL_DEFINITIONS} )


# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()


# Libraries
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
ADD_DEFINITIONS(${QT_DEFINITIONS})
#INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_USE_FILE})

# include helper file
include( ${CGAL_USE_FILE} )

# EIGEN 3
find_package(Eigen3)
if(EIGEN3_FOUND)
   include( ${EIGEN3_USE_FILE} )
else()
   message(STATUS "NOTICE: Eigen library is not found.")
endif()


# TAUCS
find_package(TAUCS)
if(TAUCS_FOUND)

  include( ${TAUCS_USE_FILE} )
else ()
  message(STATUS "NOTICE: This directory requires the TAUCS library, and some features will not be available.")
endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()



# Avoid depreciated declaration warnings
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

# Executables target folder
set( EXECUTABLE_OUTPUT_PATH bin/ )


# List all common sources
file( GLOB allsrc src/*.cpp )


#SET(main_qt_src main.cpp MainWin.cpp)
#SET(main_qt_headers MainWin.h)
#QT4_WRAP_CPP(main_qt_headers_moc ${main_qt_headers})
#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})
#ADD_EXECUTABLE(main_qt ${main_qt_src} ${main_qt_headers_moc})
#TARGET_LINK_LIBRARIES(main_qt ${QT_LIBRARIES})

#### gui
###file( GLOB main_gui src/gui/*.cpp )
###file( GLOB gui_src src/*.cpp )
###file( GLOB gui_head src/gui/*.h )
####file( GLOB gui_head src/gui/MainWin.h )
###QT4_WRAP_CPP(gui_head_moc ${gui_head})
###add_executable(gui ${main_gui} ${gui_src} ${gui_head_moc})
###target_link_libraries(gui ${OpenCV_LIBS} ${PCL_LIBRARIES} ${QT_LIBRARIES})

# scan
file( GLOB main_scan src/scan/*.cpp )
add_executable( scan ${main_scan} ${allsrc} )
target_link_libraries( scan ${OpenCV_LIBS}  )

## postProcess
#file(GLOB main_postProcess src/postProcess/*.cpp )
#add_executable( postProcess ${main_postProcess} ${allsrc})
#target_link_libraries( postProcess ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# see
file(GLOB main_see src/see/*.cpp )
add_executable( see ${main_see} ${allsrc})
target_link_libraries( see ${OpenCV_LIBS})

# surfaceReconstruction
file(GLOB main_surfaceReconstruction src/surfaceReconstruction/*.cpp )
add_executable( surfaceReconstruction ${main_surfaceReconstruction} src/DataHandler.cpp src/CloudProcessor.cpp src/MeshMaker.cpp)
target_link_libraries( surfaceReconstruction ${OpenCV_LIBS} ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES} )







