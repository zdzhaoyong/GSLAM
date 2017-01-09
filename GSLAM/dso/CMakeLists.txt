set(CMAKE_BASE_FROM_DIR ${CMAKE_CURRENT_LIST_DIR})
set(MAKE_TYPE "shared")

get_filename_component(TARGET_NAME ${CMAKE_BASE_FROM_DIR} NAME)
string(REPLACE " " "_" TARGET_NAME ${TARGET_NAME})
set(TARGET_NAME gslam_${TARGET_NAME})

set(MODULES PI_BASE EIGEN3 QT QGLVIEWER)
set(LINKFLAGS boost_system boost_thread csparse cxsparse)
set(COMPILEFLAGS ${CMAKE_CURRENT_LIST_DIR})

include(${CMAKE_CURRENT_LIST_DIR}/../../scripts/CMakeListBase.cmake)
