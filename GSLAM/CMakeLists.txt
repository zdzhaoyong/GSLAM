file(GLOB APPS2BUILD "*")
message("APPS2BUILD: " ${APPS2BUILD})

include_directories(${CMAKE_CURRENT_LIST_DIR})

foreach(APPNAME ${APPS2BUILD})
	if(IS_DIRECTORY ${APPNAME})
		add_subdirectory(${APPNAME})
	endif()
endforeach()
