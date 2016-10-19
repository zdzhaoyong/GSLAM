# PIL greator than 1.1.0

IF(WIN32)

	FIND_PATH( PIL_PATH src/base/PIL_VERSION.h
		$ENV{PIL_HOME}
		C:/PIL-1.1.0/
	)
	
	if( PIL_PATH )
		MESSAGE( STATUS "Looking for PIL-1.1.0 or greater - found")
		MESSAGE( STATUS "PIL_PATH: ${PIL_PATH}" )
		SET ( PIL_FOUND 1 )
		
		# test for 64 or 32 bit
		if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			SET( BUILD_DIR ${PIL_PATH}/build/x64 )
		else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
			SET( BUILD_DIR ${PIL_PATH}/build/x86 )
		endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
		
		# MINGW
		if(MINGW)
			SET(PIL_LIBRARIES ${BUILD_DIR}/mingw/lib/)
			file(GLOB PIL_LIBS "${PIL_LIBRARIES}*[0-9][0-9][0-9].dll.a")
		endif(MINGW)
		
		# Visual Studio 9
		if(MSVC90)
			SET(PIL_LIBRARIES ${BUILD_DIR}/vc9/lib/)
			file(GLOB PIL_RELEASE_LIBS "${PIL_LIBRARIES}*[0-9][0-9][0-9].lib")
			file(GLOB PIL_DEBUG_LIBS "${PIL_LIBRARIES}*[0-9][0-9][0-9]d.lib")
		endif(MSVC90)
		
		# Visual Studio 10
		if(MSVC10)
			SET(PIL_LIBRARIES ${BUILD_DIR}/vc10/lib/)
			file(GLOB PIL_RELEASE_LIBS "${PIL_LIBRARIES}*[0-9][0-9][0-9].lib")
			file(GLOB PIL_DEBUG_LIBS "${PIL_LIBRARIES}*[0-9][0-9][0-9]d.lib")
		endif(MSVC10)

		# Set the includes
		SET(PIL_INCLUDES ${PIL_PATH}/src ${PIL_PATH}/src)
	else( PIL_PATH )
		message( STATUS "Looking for PIL-1.1.0 or greater  - not found" )
		SET ( PIL_FOUND 0 )
	endif( PIL_PATH )

ELSE(WIN32) # Linux
	FIND_PATH( PIL_PATH src/base/PIL_VERSION.h
	# installation selected by user
	$ENV{PIL_PATH}
	# system placed in /usr/local/include
	#${PROJECT_SOURCE_DIR}/ThirdParty/PIL-1.1.0
	${PROJECT_SOURCE_DIR}/ThirdParty/PIL2
	#/data/zhaoyong/Linux/Program/Apps/PIL-1.1.0	
	#/mnt/server0/users/zhaoyong/Program/Apps/PIS-1.1.0/Thirdparty/PIL-1.1.0
	#/data/zhaoyong/Linux/Program/Apps/PIS-1.1.0/Thirdparty/PIL-1.1.0	
	)

	if(PIL_PATH)
		MESSAGE( STATUS "Found PIL at path " ${PIL_PATH})
		set(PIL_INCLUDES ${PIL_PATH}/src)
		set(PIL_MODULES2FIND base network hardware gui cv)
		foreach (PIL_MODULE_NAME ${PIL_MODULES2FIND})

			#MESSAGE( STATUS "Looking for PIL module '${PIL_MODULE_NAME}'" )
			FIND_LIBRARY(PI_${PIL_MODULE_NAME}_LIBRARIES NAMES pi_${PIL_MODULE_NAME}
				PATHS
				${PIL_PATH}/libs
				)
			#MESSAGE( STATUS "Found PIL module '${PIL_MODULE_NAME}' at ${PI_${PIL_MODULE_NAME}_LIBRARIES}" )

			if(PI_${PIL_MODULE_NAME}_LIBRARIES)
				set(PI_${PIL_MODULE_NAME}_INCLUDES ${PIL_INCLUDES})
				set(PI_${PIL_MODULE_NAME}_FOUND 1)
				list(APPEND PIL_LIBRARIES ${PI_${PIL_MODULE_NAME}_LIBRARIES})
			else(${PIL_MODULE_NAME}_LIBRARIES)
				message("Can't found module " ${PIL_MODULE_NAME})
			endif(PI_${PIL_MODULE_NAME}_LIBRARIES)

		endforeach()

	endif(PIL_PATH)

	set(PI_BASE_INCLUDES ${PI_base_INCLUDES})
	set(PI_CV_INCLUDES ${PI_base_INCLUDES})
	set(PI_NETWORK_INCLUDES ${PI_base_INCLUDES})
	set(PI_HARDWARE_INCLUDES ${PI_base_INCLUDES})
	set(PI_GUI_INCLUDES ${PI_base_INCLUDES})
	set(PI_BASE_LIBRARIES ${PI_base_LIBRARIES} -lpthread)
	set(PI_CV_LIBRARIES ${PI_cv_LIBRARIES})
	set(PI_NETWORK_LIBRARIES ${PI_network_LIBRARIES})
	set(PI_HARDWARE_LIBRARIES ${PI_hardware_LIBRARIES})
	set(PI_GUI_LIBRARIES ${PI_gui_LIBRARIES})

		if( PIL_INCLUDES AND PIL_LIBRARIES)
			MESSAGE( STATUS "Looking for PIL-1.1.0 or greater - found")
			MESSAGE( STATUS "PIL include path: ${PIL_INCLUDES}" )
			SET ( PIL_FOUND 1 )
		else(  PIL_INCLUDES AND PIL_LIBRARIES)
			message( STATUS "Looking for PIL-1.1.0 or greater  - not found" )
			SET ( PIL_FOUND 0 )
			
			message( SEND_ERROR "Please clone PIL (https://github.com/zdzhaoyong/PIL2) to the ThirdParty (${PROJECT_SOURCE_DIR}/ThirdParty) and compile it first!")
			
				
		endif(  PIL_INCLUDES AND PIL_LIBRARIES)
	
ENDIF(WIN32)
