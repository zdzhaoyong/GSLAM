# Need to find both Qt4 and QGLViewer if the QQL support is to be built
#FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui)

# this cmake module force to use static version of libQQLViewer
# please setup "QGLViewer_DIR" environment variable
#

FIND_PATH(QGLVIEWER_INC1 qglviewer.h
    $ENV{QGLViewer_DIR}/include/QGLViewer
    /opt/libQGLViewer/include/QGLViewer
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    /usr/local/include/QGLViewer
    /sw/include/QGLViewer
    )

get_filename_component(QGLVIEWER_INC2 "${QGLVIEWER_INC1}/../" ABSOLUTE)
SET(QGLVIEWER_INCLUDES ${QGLVIEWER_INC1} ${QGLVIEWER_INC2})

if(QT4_FOUND)
    set(QGL_LIBNAMES QGLViewer-qt4 qglviewer-qt4)
else(QT5_FOUND)
    set(QGL_LIBNAMES QGLViewer QGLViewer2)
endif()

FIND_LIBRARY(QGLVIEWER_LIBRARIES NAMES ${QGL_LIBNAMES}
    PATHS
    $ENV{QGLViewer_DIR}/lib
    /opt/libQGLViewer/lib
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    )

# FIXME: if under windows, use static lib
IF( WIN32 )
    SET(QGLVIEWER_DEFINITIONS -DQGLVIEWER_STATIC)
ENDIF( WIN32 )


IF(QGLVIEWER_INCLUDES AND QGLVIEWER_LIBRARIES)
    SET(QGLVIEWER_FOUND TRUE)
    MESSAGE("-- Found QGLViewer: ${QGLVIEWER_INCLUDES}")
ELSE(QGLVIEWER_INCLUDES AND QGLVIEWER_LIBRARIES)
    SET(QGLVIEWER_FOUND FALSE)
ENDIF(QGLVIEWER_INCLUDES AND QGLVIEWER_LIBRARIES)


