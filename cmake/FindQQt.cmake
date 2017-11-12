set(QT_QMAKE_EXECUTABLE /mingw64/bin/qmake)
set(QT_QTCORE_INCLUDE_DIR /mingw64/include/qt4/QtCore)
#set(CMAKE_PREFIX_PATH /mingw64/bin)
find_package(Qt4)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${QT_INCLUDE_DIR})
set(QT_LIBRARIES QtGui QtCore QtXml QtOpenGL)

