find_package(Qt4)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${QT_INCLUDE_DIR})

if( WIN32)     
    set(QT_LIBRARIES QtGui4 QtCore4 QtXml4 QtOpenGL4 QtSql4 QtNetwork4 QtSvg4)
    
    # FIXME: 
    set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /mingw64/lib)
	
	message(STATUS "QT_LIBRARIES = ${QT_LIBRARIES}")
else( WIN32 )
    set(QT_LIBRARIES QtGui QtCore QtXml QtOpenGL QtSql QtNetwork QtSvg)
endif( WIN32)

set(QT_FOUND 1)
