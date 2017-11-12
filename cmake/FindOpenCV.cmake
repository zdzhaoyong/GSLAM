# OPENCV2_LIBS the list of OpenCV 2.2 or greater libs (WIN32 MINGW compiler only)

set(OPENCV_STATICLIB 1)

FIND_PATH( OPENCV_INCLUDE_DIR opencv2/opencv.hpp
    # installation selected by user
    $ENV{OPENCV_HOME}/include
    $ENV{OpenCV_DIR}/include
    # system placed in /usr/local/include
    /usr/local/include
    # system placed in /usr/include
    /usr/include
    /opt/opencv-2.4.9/include
    )

message(STATUS "OPENCV_INCLUDE_DIR = ${OPENCV_INCLUDE_DIR}")

if(WIN32)
    set(OPENCV_MODULES2FIND opencv_highgui249 opencv_calib3d249
        opencv_features2d249 opencv_ocl249
        opencv_legacy249 opencv_superres249 opencv_videostab249 opencv_ml249
        opencv_contrib249 opencv_flann249 opencv_photo249
        opencv_objdetect249 opencv_stitching249 opencv_imgcodecs249 opencv_gpu249
        opencv_imgproc249 opencv_core249)
        
    if( OPENCV_STATICLIB )
        list(APPEND OPENCV_MODULES2FIND  IlmImf libjasper libjpeg libpng libtiff zlib)
    endif( OPENCV_STATICLIB )
else(WIN32)
    set(OPENCV_MODULES2FIND opencv_highgui  opencv_calib3d
        opencv_features2d 
        opencv_legacy opencv_superres opencv_videostab opencv_ml
        opencv_contrib opencv_flann opencv_photo opencv_objdetect
        opencv_stitching opencv_imgcodecs
        opencv_imgproc
        opencv_ocl opencv_gpu 
        opencv_core)

        
    if( OPENCV_STATICLIB )
        list(APPEND OPENCV_MODULES2FIND  IlmImf libjasper libjpeg libpng libtiff zlib)
    endif( OPENCV_STATICLIB )
endif(WIN32)

message(STATUS "OPENCV_MODULES2FIND = ${OPENCV_MODULES2FIND}")

foreach (OPENCV_MODULE_NAME ${OPENCV_MODULES2FIND})
    FIND_LIBRARY(${OPENCV_MODULE_NAME}_LIBRARIES NAMES ${OPENCV_MODULE_NAME}
        PATHS
        $ENV{OPENCV_HOME}/x64/mingw/lib
        $ENV{OPENCV_HOME}/x64/mingw/staticlib
        $ENV{OpenCV_DIR}/lib
        /opt/opencv-2.4.9/lib
        /opt/opencv-2.4.9/share/OpenCV/3rdparty/lib
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        NO_DEFAULT_PATH
        NO_SYSTEM_ENVIRONMENT_PATH
        )

    if(${OPENCV_MODULE_NAME}_LIBRARIES)
        set(${OPENCV_MODULE_NAME}_INCLUDES ${OPENCV_INCLUDES})
        set(${OPENCV_MODULE_NAME}_FOUND 1)
        list(APPEND OPENCV_LIBRARIES ${${OPENCV_MODULE_NAME}_LIBRARIES})
    else(${OPENCV_MODULE_NAME}_LIBRARIES)
        message("Can't found module " ${OPENCV_MODULE_NAME})
    endif(${OPENCV_MODULE_NAME}_LIBRARIES)
endforeach()

if( OPENCV_INCLUDE_DIR AND OPENCV_LIBRARIES )
    MESSAGE( STATUS "Looking for OpenCV2.2 or greater - found")
    MESSAGE( STATUS "OpenCV2.2 include path: ${OPENCV_INCLUDE_DIR}" )
    #MESSAGE( STATUS "OpenCV2.2 LIBS: ${OPENCV_LIBRARIES}" )
    SET ( OPENCV2_FOUND 1 )
    SET ( OPENCV_FOUND 1 )
else(  OPENCV_INCLUDE_DIR AND OPENCV_LIBRARIES )
    message( STATUS "Looking for OpenCV2.2 or greater  - not found" )
    SET ( OPENCV2_FOUND 0 )
    SET ( OPENCV_FOUND 0 )
endif(  OPENCV_INCLUDE_DIR AND OPENCV_LIBRARIES )


IF(OPENCV2_FOUND)
    set(OpenCV_INCLUDES ${OPENCV_INCLUDE_DIR})
    set(OpenCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIR})
    
    set(OpenCV_LIBRARIES ${OPENCV_LIBRARIES})
    set(OpenCV_LIBS ${OPENCV_LIBRARIES})
ENDIF(OPENCV2_FOUND)

