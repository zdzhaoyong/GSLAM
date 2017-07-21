IF(WIN32)
		message( STATUS "Looking for G2O or greater  - not found" )
		SET ( PIL_FOUND 0 )
ELSE(WIN32) # Linux

#	message("G2O path?=" ${PROJECT_SOURCE_DIR}/ThirdParty/g2o)
	FIND_PATH( G2O_PATH g2o/types/sim3.h
	${PROJECT_SOURCE_DIR}/ThirdParty/g2o
	/data/zhaoyong/Linux/Program/Apps/Thirdparty/orbslam/trunk/Thirdparty/g2o
	# installation selected by user
	$ENV{G2O_PATH}
	# system placed in /usr/local/include
	)

	if(G2O_PATH)
	MESSAGE( STATUS "Found g2o at path " ${G2O_PATH})
	set(G2O_INCLUDES ${G2O_PATH})
			FIND_LIBRARY(G2O_LIBRARIES NAMES g2o
				PATHS
				${G2O_PATH}/lib
				)
	endif(G2O_PATH)

		if( G2O_INCLUDES AND G2O_LIBRARIES)
			MESSAGE( STATUS "Looking for G2O or greater - found")
			SET ( G2O_FOUND 1 )
		else()
			message( STATUS "Looking for G2O or greater  - not found" )
			SET ( G2O_FOUND 0 )
		endif()
	
ENDIF(WIN32)
