set(GLOG_FOUND 1)

FIND_PATH(GLOG_INCLUDES glog/logging.h
    $ENV{GLog_DIR}/include
    C:/msys64/usr/local/include
    /usr/local/include
)

FIND_LIBRARY(GLOG_LIBRARIES NAMES glog
    PATHS
    $ENV{GLog_DIR}/lib
    C:/msys64/usr/local/lib
    /usr/local/lib
    #NO_DEFAULT_PATH
    #NO_SYSTEM_ENVIRONMENT_PATH
)

message(STATUS "GLOG_LIBRARIES = ${GLOG_LIBRARIES}")

if( WIN32)     
    #set(GLOG_LIBRARIES C:/msys64/usr/local/lib/libglog.a )
	#set(GLOG_INCLUDES C:/msys64/usr/local/include)
    set(GLOG_DEFINITIONS -DGOOGLE_GLOG_DLL_DECL=)
else( WIN32 )
    #set(GLOG_LIBRARIES glog)
endif( WIN32)
