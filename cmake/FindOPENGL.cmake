set(OpenGL_FOUND 1)

if( WIN32)     
    #set(OPENGL_LIBRARIES opengl32 glu32 freeglut glew32)
    set(OPENGL_LIBRARIES opengl32 glu32 freeglut_static)
    
    # FIXME: 
    set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /mingw64/lib)
else( WIN32 )
    set(OPENGL_LIBRARIES GL GLU glut GLEW)
endif( WIN32)

