cmake_minimum_required(VERSION 2.8.12)
project(svarpy)

set(CMAKE_CXX_STANDARD 11)

include(../../cmake/PICMake.cmake)
include_directories(../..)

set(Python_ADDITIONAL_VERSIONS 3.7 3.6 3.5 3.4)

pi_add_target(gslam SHARED src REQUIRED Python System LTO)

set_target_properties(gslam PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}")
set_target_properties(gslam PROPERTIES SUFFIX "${PYTHON_MODULE_EXTENSION}")

pi_report_target()
