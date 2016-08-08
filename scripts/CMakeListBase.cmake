####################################################################
## BASIC CONFIGURATIONS
####################################################################
# 1. Get folder name
if(NOT TARGET_NAME)
get_filename_component(TARGET_NAME ${CMAKE_BASE_FROM_DIR} NAME)
string(REPLACE " " "_" TARGET_NAME ${TARGET_NAME})
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BASE_FROM_DIR)
	cmake_minimum_required(VERSION 2.8.6)
	project(${TARGET_NAME})
endif()

# 2. Pasing configuration

if(NOT MAKE_TYPE)
set(MAKE_TYPE "bin")
endif()
# set(MODULES "")

if(EXISTS "config.mk")

	message("Parsing config.mk at folder " ${CMAKE_BASE_FROM_DIR})
	set(MODULES ${MODULES} QT QGLVIEWER PI_BASE PI_GUI)

endif()


# 3. Collect all source files and dependencies
file(GLOB_RECURSE SOURCE_FILES_ALL RELATIVE ${CMAKE_BASE_FROM_DIR} *.cpp *.c *.cc)

if(SOURCE_FILES_ALL)

	foreach(MODULE_NAME ${MODULES})
		list(APPEND COMPILEFLAGS ${${MODULE_NAME}_INCLUDES})
		list(APPEND LINKFLAGS    ${${MODULE_NAME}_LIBRARIES})
	endforeach()

	include_directories(${COMPILEFLAGS})

	if(MAKE_TYPE STREQUAL "bin")
		set_property( GLOBAL APPEND PROPERTY APPS2COMPILE  " ${TARGET_NAME}")
		add_executable(${TARGET_NAME} ${CMAKE_BASE_FROM_DIR} ${SOURCE_FILES_ALL})
	elseif(MAKE_TYPE STREQUAL "static")
		set_property( GLOBAL APPEND PROPERTY LIBS2COMPILE  " ${CMAKE_STATIC_LIBRARY_PREFIX}${TARGET_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX}")
		add_library(${TARGET_NAME} STATIC ${CMAKE_BASE_FROM_DIR} ${SOURCE_FILES_ALL})
	elseif(MAKE_TYPE STREQUAL "shared")
		set_property( GLOBAL APPEND PROPERTY LIBS2COMPILE  " ${CMAKE_SHARED_LIBRARY_PREFIX}${TARGET_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
		add_library(${TARGET_NAME} SHARED ${CMAKE_BASE_FROM_DIR} ${SOURCE_FILES_ALL})
	else()
		message ("No such make type: " ${MAKE_TYPE})
	endif()

	target_link_libraries(${TARGET_NAME} ${LINKFLAGS})
	if(MOC_FILES)
		set_target_properties(${TARGET_NAME} PROPERTIES AUTOMOC TRUE)
	endif()
endif()
