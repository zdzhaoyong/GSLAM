FIND_PATH(FFMPEG_INCLUDES 
	NAMES	libavcodec/avcodec.h
	PATHS
	${PROJECT_SOURCE_DIR}/../ffmpeg_mingw/include/ffmpeg_
	/usr/include/x86_64-linux-gnu/
	/usr/include/
	/usr/local/include/
	C:/msys64/mingw64/include/	
	NO_DEFAULT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
)

if(WIN32)
	set(FFMPEG_LIBPATH C:/msys64/mingw64/lib/)
	set(FFMPEG_LIBPATH C:/msys64/home/Administrator/RTMapper/ffmpeg_mingw/lib)
	
	set(AVSCALE  ${FFMPEG_LIBPATH}/libswscale.a)
	set(AVUTIL   ${FFMPEG_LIBPATH}/libavutil.a)
	set(AVCODEC  ${FFMPEG_LIBPATH}/libavcodec.a)
	set(AVFORMAT ${FFMPEG_LIBPATH}/libavformat.a)

	set(AVSCALE  ${FFMPEG_LIBPATH}/libswscale64.a)
	set(AVUTIL   ${FFMPEG_LIBPATH}/libavutil64.a)
	set(AVCODEC  ${FFMPEG_LIBPATH}/libavcodec64.a)
	set(AVFORMAT ${FFMPEG_LIBPATH}/libavformat64.a)
	
	
	FIND_LIBRARY(AVSCALE NAMES swscale64
    PATHS
    ${PROJECT_SOURCE_DIR}/../ffmpeg_mingw/lib
    )
	
	FIND_LIBRARY(AVUTIL NAMES avutil64
    PATHS
    ${PROJECT_SOURCE_DIR}/../ffmpeg_mingw/lib
    )
	
	FIND_LIBRARY(AVCODEC NAMES avcodec64
    PATHS
    ${PROJECT_SOURCE_DIR}/../ffmpeg_mingw/lib
    )
	
	FIND_LIBRARY(AVFORMAT NAMES avformat64
    PATHS
    ${PROJECT_SOURCE_DIR}/../ffmpeg_mingw/lib
    )
	
else(WIN32)

	FIND_LIBRARY(AVSCALE
			NAMES swscale
			 PATHS
			/usr/lib/x86_64-linux-gnu/
			/usr/lib/
			/usr/local/lib/
			C:/msys64/mingw64/lib
	)
	FIND_LIBRARY(AVUTIL
			NAMES avutil 
			PATHS
			/usr/lib/x86_64-linux-gnu/
			/usr/lib/
			/usr/local/lib/
		  C:/msys64/mingw64/lib
	)
	FIND_LIBRARY(AVCODEC
			NAMES avcodec 
			PATHS
			/usr/lib/x86_64-linux-gnu/
			/usr/lib/
			/usr/local/lib/
		  C:/msys64/mingw64/lib
	)
	FIND_LIBRARY(AVFORMAT
			NAMES avformat 
			PATHS
			/usr/lib/x86_64-linux-gnu/
			/usr/lib/
			/usr/local/lib/
		  C:/msys64/mingw64/lib
	)
	FIND_LIBRARY(AVDEVICE
			NAMES avdevice
			PATHS
			/usr/lib/x86_64-linux-gnu/
			/usr/lib/
			/usr/local/lib/
			C:/msys64/mingw64/lib
	)
endif(WIN32)


#-lswresample -lswscale -lavfilter -lavdevice -lavformat -lavcodec -lavutil 
LIST(APPEND FFMPEG_LIB ${AVFORMAT} ${AVCODEC} ${AVSCALE}  ${AVUTIL} )

set(FFMPEG_LIBRARIES ${FFMPEG_LIB})
set(FFMPEG_INCLUDE_DIR ${FFMPEG_INCLUDES})
set(FFMPEG_FOUND true)
