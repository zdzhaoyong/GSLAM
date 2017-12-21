cmake_minimum_required(VERSION 2.8.6)
project(GSLAM)
set(CMAKE_FIND_ROOT_PATH C:/msys64)

macro(GSLAM_FIND_VERSION)
  file(READ "${PROJECT_SOURCE_DIR}/GSLAM/core/GSLAM.h" _gslam_version_header)

  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_MAJOR[ \t]+([0-9]+)" _gslam_major_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_MAJOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_MINOR[ \t]+([0-9]+)" _gslam_minor_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_MINOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_PATCH[ \t]+([0-9]+)" _gslam_patch_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_PATCH "${CMAKE_MATCH_1}")
	
  if(NOT GSLAM_VERSION_MAJOR)
    set(GSLAM_VERSION_MAJOR 0)
  endif(NOT GSLAM_VERSION_MAJOR)
  if(NOT GSLAM_VERSION_MINOR)
    set(GSLAM_VERSION_MINOR 0)
  endif(NOT GSLAM_VERSION_MINOR)
  if(NOT GSLAM_VERSION_PATCH)
    set(GSLAM_VERSION_PATCH 0)
  endif(NOT GSLAM_VERSION_PATCH)
	

  set(GSLAM_VERSION ${GSLAM_VERSION_MAJOR}.${GSLAM_VERSION_MINOR}.${GSLAM_VERSION_PATCH})
  set(PROJECT_VERSION ${GSLAM_VERSION})
  set(PROJECT_SOVERSION ${GSLAM_VERSION_MAJOR}.${GSLAM_VERSION_MINOR})
endmacro()
GSLAM_FIND_VERSION()
message("-- Compiling GSLAM Version ${GSLAM_VERSION}.")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)#Debug
ENDIF()

message("-- Build type: " ${CMAKE_BUILD_TYPE})

if(CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
endif()

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


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(PICMake)

#####################################################################
# Find dependencies
message("----------------------------------------------------------")

# The GSLAM core need only -std-c++11 and no extra dependency, to build gslam with Util, GUI and IO things, please install the following dependencies.
pi_collect_packages(VERBOSE REQUIRED System MODULES # System dependency -dl is essential
                    Eigen3 Ceres                    # Dependency for Optimizer
                    Qt OpenGL GLEW GLUT QGLViewer   # Dependency for GUI
                    OpenCV                          # Dependency for IO
                    )

message("----------------------------------------------------------")

#####################################################################
# Now do build things
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
if(QT_FOUND)
qt4_add_resources(QRC_FILES GSLAM/gslam/GUI/res.qrc)
endif()

include_directories(.)

pi_add_target(gslam BIN GSLAM/gslam ${QRC_FILES} REQUIRED System MODULES OpenCV Qt OpenGL GLEW GLUT QGLViewer)
pi_add_target(gslamDB_cfg SHARED GSLAM/plugins/cfg REQUIRED System MODULES OpenCV Qt)
pi_add_target(gslam_optimizer SHARED GSLAM/plugins/optimizer_ceres REQUIRED System Eigen3 Ceres)
pi_add_target(gslam_estimator SHARED GSLAM/plugins/estimator_opencv REQUIRED OpenCV)

pi_report_target()
#####################################################################
# Now do make install
pi_install(HEADERS GSLAM/core HEADER_DESTINATION include/GSLAM
           TARGETS gslam gslamDB_cfg gslam_optimizer gslam_estimator CMAKE "${PROJECT_SOURCE_DIR}/cmake/FindGSLAM.cmake.in")

