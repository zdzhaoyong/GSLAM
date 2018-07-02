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

if(UNIX)
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
elseif(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DNOMINMAX")
endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(PICMake)

#####################################################################
# Find dependencies
message("----------------------------------------------------------")

# The GSLAM core need only -std-c++11 and no extra dependency, to build gslam with Util, GUI and IO things, please install the following dependencies.
pi_collect_packages(VERBOSE REQUIRED System MODULES # System dependency -dl is essential
                    Eigen3 Ceres                    # Dependency for Optimizer
                    Qt OpenGL                       # Dependency for GUI
                    OpenCV                          # Dependency for IO
                    )

message("----------------------------------------------------------")

#####################################################################
# Now do build things
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
if(QT4_FOUND)
    qt4_add_resources(QRC_FILES GSLAM/gslam/GUI/res.qrc)
elseif(QT5_FOUND)
    qt5_add_resources(QRC_FILES GSLAM/gslam/GUI/res.qrc)
endif()

include_directories(. GSLAM/core)

pi_add_target(gslam BIN GSLAM/gslam ${QRC_FILES} REQUIRED System MODULES Qt OpenGL)

pi_add_target(gslam_optimizer SHARED GSLAM/plugins/optimizer_ceres REQUIRED System Eigen3 Ceres GLOG)
pi_add_target(gslam_estimator SHARED GSLAM/plugins/estimator_opencv REQUIRED OpenCV)

pi_add_target(gslamDB_euroc SHARED GSLAM/gslam/IO/DatasetEuroc.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_kitti SHARED GSLAM/gslam/IO/DatasetKITTI.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_npurgbd SHARED GSLAM/gslam/IO/DatasetNPURGBD.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_cvmono SHARED GSLAM/gslam/IO/DatasetOpenCVMono.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_tummono SHARED GSLAM/gslam/IO/DatasetTUMMono.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_tumrgbd SHARED GSLAM/gslam/IO/DatasetTUMRGBD.cpp REQUIRED OpenCV)


pi_report_target()
#####################################################################
# Now do make install
pi_install(HEADERS GSLAM/core HEADER_DESTINATION include/GSLAM
           TARGETS gslam gslamDB_cfg gslam_optimizer gslam_estimator CMAKE "${PROJECT_SOURCE_DIR}/cmake/FindGSLAM.cmake.in")

