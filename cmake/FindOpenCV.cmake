# OPENCV2_LIBS the list of OpenCV 2.2 or greater libs (WIN32 MINGW compiler only)

#include(/opt/opencv-3.2/share/OpenCV/OpenCVConfig.cmake)
#set(OpenCV_INCLUDES ${OpenCV_INCLUDE_DIRS})
#return()

IF(WIN32)

	FIND_PATH( OPENCV2_PATH include/opencv2/opencv.hpp
		$ENV{OPENCV_HOME}
		C:/OpenCV2.2/
		C:/OpenCV2.3/
		C:/OpenCV2.4/
	)
	
	if( OPENCV2_PATH )
		MESSAGE( STATUS "Looking for OpenCV2.2 or greater - found")
		MESSAGE( STATUS "OpenCV2.2 path: ${OPENCV2_PATH}" )
		SET ( OPENCV2_FOUND 1 )
		
		# test for 64 or 32 bit
		if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			SET( BUILD_DIR ${OPENCV2_PATH}/build/x64 )
		else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			SET( BUILD_DIR ${OPENCV2_PATH}/build/x86 )
		endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
		
		# MINGW
		if(MINGW)
			SET(OPENCV2_LIB_PATH ${BUILD_DIR}/mingw/lib/)
			file(GLOB OPENCV2_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9].dll.a")
		endif(MINGW)
		
		# Visual Studio 9
		if(MSVC90)
			SET(OPENCV2_LIB_PATH ${BUILD_DIR}/vc9/lib/)
			file(GLOB OPENCV2_RELEASE_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9].lib")
			file(GLOB OPENCV2_DEBUG_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9]d.lib")
		endif(MSVC90)
		
		# Visual Studio 10
		if(MSVC10)
			SET(OPENCV2_LIB_PATH ${BUILD_DIR}/vc10/lib/)
			file(GLOB OPENCV2_RELEASE_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9].lib")
			file(GLOB OPENCV2_DEBUG_LIBS "${OPENCV2_LIB_PATH}*[0-9][0-9][0-9]d.lib")
		endif(MSVC10)

		# Set the includes
		SET(OPENCV2_INCLUDE_PATH ${OPENCV2_PATH}/build/include/opencv2 ${OPENCV2_PATH}/build/include)
		

	else( OPENCV2_PATH )
		message( STATUS "Looking for OpenCV2.2 or greater  - not found" )
		SET ( OPENCV2_FOUND 0 )
	endif( OPENCV2_PATH )

ELSE(WIN32) # Linux

	FIND_PATH( OPENCV_INCLUDE_DIR opencv2/opencv.hpp
	# installation selected by user
	$ENV{OPENCV_HOME}/include
	# system placed in /usr/local/include
	/usr/local/include
	# system placed in /usr/include
	/usr/include
	/opt/opencv-2.4.9/include
	)


	set(OPENCV_MODULES2FIND opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_features2d opencv_ocl opencv_gpu opencv_legacy opencv_superres opencv_videostab opencv_ml opencv_contrib opencv_flann opencv_photo opencv_objdetect opencv_stitching opencv_imgcodecs)

	foreach (OPENCV_MODULE_NAME ${OPENCV_MODULES2FIND})
		FIND_LIBRARY(${OPENCV_MODULE_NAME}_LIBRARIES NAMES ${OPENCV_MODULE_NAME}
			PATHS
			/usr/lib/x86_64-linux-gnu
			/usr/lib
			/usr/local/lib
			/opt/local/lib
			/sw/lib
			/opt/opencv-2.4.9/lib
			)
		if(${OPENCV_MODULE_NAME}_LIBRARIES)
			set(${OPENCV_MODULE_NAME}_INCLUDES ${OPENCV_INCLUDES})
			set(${OPENCV_MODULE_NAME}_FOUND 1)
			list(APPEND OPENCV_LIBRARIES ${${OPENCV_MODULE_NAME}_LIBRARIES})
		else(${OPENCV_MODULE_NAME}_LIBRARIES)
			message("Can't found module " ${OPENCV_MODULE_NAME})
		endif(${OPENCV_MODULE_NAME}_LIBRARIES)
	endforeach()

	if( OPENCV_INCLUDE_DIR AND OPENCV_LIBRARIES)
		MESSAGE( STATUS "Looking for OpenCV2.2 or greater - found")
		MESSAGE( STATUS "OpenCV2.2 include path: ${OPENCV_INCLUDE_DIR}" )
		#MESSAGE( STATUS "OpenCV2.2 LIBS: ${OPENCV_LIBRARIES}" )
		SET ( OPENCV2_FOUND 1 )
		SET ( OPENCV_FOUND 1 )
	else(  OPENCV_INCLUDE_DIR AND OPENCV_LIBRARIES)
		message( STATUS "Looking for OpenCV2.2 or greater  - not found" )
		SET ( OPENCV2_FOUND 0 )
		SET ( OPENCV_FOUND 0 )
	endif(  OPENCV_INCLUDE_DIR AND OPENCV_LIBRARIES)

	
ENDIF(WIN32)

IF(OPENCV2_FOUND)
	  	set(OpenCV_INCLUDES ${OPENCV_INCLUDE_DIR})
	  	set(OpenCV_LIBRARIES ${OPENCV_LIBRARIES})
ENDIF(OPENCV2_FOUND)
