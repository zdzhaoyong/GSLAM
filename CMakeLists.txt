cmake_minimum_required(VERSION 2.8.6)
project(GSLAM)

macro(GSLAM_FIND_VERSION)
  file(READ "${PROJECT_SOURCE_DIR}/GSLAM/core/GSLAM.h" _gslam_version_header)

  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_MAJOR[ \t]+([0-9]+)" _gslam_major_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_MAJOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_MINOR[ \t]+([0-9]+)" _gslam_minor_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_MINOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_PATCH[ \t]+([0-9]+)" _gslam_patch_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_PATCH "${CMAKE_MATCH_1}")
	
  if(NOT GSLAM_VERSION_MAJOR)
    set(GSLAM_VERSION_MAJOR 1)
  endif(NOT GSLAM_VERSION_MAJOR)
  if(NOT GSLAM_VERSION_MINOR)
    set(GSLAM_VERSION_MINOR 1)
  endif(NOT GSLAM_VERSION_MINOR)
  if(NOT GSLAM_VERSION_PATCH)
    set(GSLAM_VERSION_PATCH 0)
  endif(NOT GSLAM_VERSION_PATCH)
	

  set(GSLAM_VERSION ${GSLAM_VERSION_MAJOR}.${GSLAM_VERSION_MINOR}.${GSLAM_VERSION_PATCH})
  set(PROJECT_VERSION ${GSLAM_VERSION})
  set(PROJECT_SOVERSION ${GSLAM_VERSION_MAJOR}.${GSLAM_VERSION_MINOR})
endmacro()

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


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/scripts/cmake)
include(PICMake)

#####################################################################
# Find dependencies
message("----------------------------------------------------------")

#**Qt** : sudo apt-get install build-essential g++ libqt4-core libqt4-dev libqt4-gui qt4-doc qt4-designer 
#**QGLViewer** : sudo apt-get install libqglviewer-dev libqglviewer2

find_package(PIL 1.1.1 REQUIRED)
find_package(OpenCV 2.4.9)
find_package(Eigen3 3.1.0)
find_package(Qt)
pi_report_modules(PIL OpenCV Eigen3 Qt)
pi_collect_packages(VERBOSE MODULES OpenGL GLEW GLUT G2O QGLViewer Boost)


GSLAM_FIND_VERSION()
message("----------------------------------------------------------")

#####################################################################
# Now do build things
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# Build libs
include_directories(${CMAKE_CURRENT_LIST_DIR} ${CMAKE_SOURCE_DIR}/GSLAM/dso)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath,.:./libs:../libs:${CMAKE_INSTALL_PREFIX}/lib")

pi_add_target(gslam_core SHARED GSLAM/core REQUIRED PIL PI_BASE)
pi_add_target(gslam_dso SHARED GSLAM/dso REQUIRED gslam_core PIL PI_BASE BOOST EIGEN3 QT QGLVIEWER GLEW GLUT) 
pi_add_target(gslam_orbslam SHARED GSLAM/orbslam REQUIRED gslam_core PIL PI_BASE BOOST EIGEN3 QT QGLVIEWER GLEW GLUT G2O OpenCV)
pi_add_target(gslam BIN GSLAM/gslam REQUIRED gslam_core PI_BASE OpenCV Qt QGLViewer)


pi_report_target()


#####################################################################
# Now do make install
pi_install(HEADERS GSLAM/core HEADER_DESTINATION include/GSLAM
           TARGETS gslam_core gslam CMAKE "${PROJECT_SOURCE_DIR}/scripts/cmake/FindGSLAM.cmake.in")

