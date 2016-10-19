cmake_minimum_required(VERSION 2.8.6)
project(GSLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)#Debug
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/scripts)
include(scripts/CMakeUtils.cmake)
#####################################################################
# Find dependencies

#**Qt** : sudo apt-get install build-essential g++ libqt4-core libqt4-dev libqt4-gui qt4-doc qt4-designer 
#**QGLViewer** : sudo apt-get install libqglviewer-dev libqglviewer2

find_package(OpenCV 2.4.9 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(QGLViewer    REQUIRED)
find_package(PIL 1.1.0 REQUIRED)
find_package(G2O REQUIRED)

set(QT_LIBRARIES QtGui QtCore QtXml QtOpenGL GL GLU glut GLEW)
include_directories(${CMAKE_CURRENT_LIST_DIR})

#####################################################################
# Results collection

message("----------------------------------------------------------")
PIL_ECHO_LIBINFO(OPENCV)
PIL_ECHO_LIBINFO(QT)
PIL_ECHO_LIBINFO(QGLVIEWER)
PIL_ECHO_LIBINFO(PIL)
PIL_ECHO_LIBINFO(EIGEN3)
PIL_ECHO_LIBINFO(G2O)

message("----------------------------------------------------------")

#####################################################################
# Now do build things
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# Build libs
add_subdirectory(GSLAM)

get_property(LIBS2COMPILE GLOBAL PROPERTY LIBS2COMPILE)
get_property(APPS2COMPILE GLOBAL PROPERTY APPS2COMPILE)
 
message(STATUS "The following targets will to be build:")
message(STATUS "LIBS(${CMAKE_LIBRARY_OUTPUT_DIRECTORY}): " ${LIBS2COMPILE})
message(STATUS "APPS(${CMAKE_RUNTIME_OUTPUT_DIRECTORY}): " ${APPS2COMPILE})
