# Need to find both Qt4 and QGLViewer if the QQL support is to be built
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui)

FIND_PATH(QGLVIEWER_INCLUDES qglviewer.h
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    /usr/local/include/QGLViewer
    /sw/include/QGLViewer
  )

FIND_LIBRARY(QGLVIEWER_LIBRARIES NAMES QGLViewer-qt4 qglviewer-qt4 QGLViewer
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )


IF(QGLVIEWER_INCLUDES AND QGLVIEWER_LIBRARIES)
  SET(QGLVIEWER_FOUND TRUE)
	MESSAGE("-- Found QGLViewer: ${QGLVIEWER_INCLUDES}")
ELSE(QGLVIEWER_INCLUDES AND QGLVIEWER_LIBRARIES)
  SET(QGLVIEWER_FOUND FALSE)
ENDIF(QGLVIEWER_INCLUDES AND QGLVIEWER_LIBRARIES)
