cmake_minimum_required(VERSION 2.8.6)
project(GSLAM)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(PICMake)
include(GSLAMVersion)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)#Debug
ENDIF()

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
endif()

#####################################################################
# Add targets
message("-- Compiling GSLAM Version ${GSLAM_VERSION}.")
message("-- Build type: " ${CMAKE_BUILD_TYPE})

include_directories(. GSLAM/core)

pi_add_target(gslam BIN GSLAM/gslam ${QRC_FILES} REQUIRED System MODULES Qt OpenGL)

pi_add_target(gslam_optimizer SHARED GSLAM/plugins/optimizer_ceres REQUIRED System Eigen3 Ceres GLOG)
pi_add_target(gslam_estimator SHARED GSLAM/plugins/estimator_opencv REQUIRED OpenCV)

pi_add_target(gslamDB_euroc SHARED GSLAM/gslam/IO/DatasetEuroc.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_kitti SHARED GSLAM/gslam/IO/DatasetKITTI.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_npudronemap SHARED GSLAM/gslam/IO/DatasetNPUDroneMap.cpp)
pi_add_target(gslamDB_npurgbd SHARED GSLAM/gslam/IO/DatasetNPURGBD.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_cvmono SHARED GSLAM/gslam/IO/DatasetOpenCVMono.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_rtm SHARED GSLAM/gslam/IO/DatasetRTMapper.cpp REQUIRED Qt)
pi_add_target(gslamDB_tummono SHARED GSLAM/gslam/IO/DatasetTUMMono.cpp REQUIRED OpenCV)
pi_add_target(gslamDB_tumrgbd SHARED GSLAM/gslam/IO/DatasetTUMRGBD.cpp REQUIRED OpenCV)

pi_report_target()

#####################################################################
# Now do make install
get_property(TARGETS2COMPILE GLOBAL PROPERTY TARGETS2COMPILE)
pi_install(HEADERS GSLAM/core HEADER_DESTINATION include/GSLAM
           TARGETS ${TARGETS2COMPILE} CMAKE "${PROJECT_SOURCE_DIR}/cmake/FindGSLAM.cmake.in")

