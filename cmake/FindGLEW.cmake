set(GLEW_FOUND 1)

if( WIN32)     
    set(GLEW_LIBRARIES C:/msys64/mingw64/lib/libglew32.a)
    set(GLEW_DEFINITIONS -DGLEW_STATIC)
else( WIN32 )
    #set(GLEW_LIBRARIES GLEW)
    set(GLEW_LIBRARIES /usr/lib64/libGLEW.a)
    set(GLEW_DEFINITIONS -DGLEW_STATIC)
endif( WIN32)

