
macro(GSLAM_FIND_VERSION)
  file(READ "${PROJECT_SOURCE_DIR}/GSLAM/core/GSLAM.h" _gslam_version_header)

  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_MAJOR[ \t]+([0-9]+)" _gslam_major_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_MAJOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_MINOR[ \t]+([0-9]+)" _gslam_minor_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_MINOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+GSLAM_VERSION_PATCH[ \t]+([0-9]+)" _gslam_patch_version_match "${_gslam_version_header}")
  set(GSLAM_VERSION_PATCH "${CMAKE_MATCH_1}")
	
  if(NOT GSLAM_VERSION_MAJOR)
    set(GSLAM_VERSION_MAJOR 0)
  endif(NOT GSLAM_VERSION_MAJOR)
  if(NOT GSLAM_VERSION_MINOR)
    set(GSLAM_VERSION_MINOR 0)
  endif(NOT GSLAM_VERSION_MINOR)
  if(NOT GSLAM_VERSION_PATCH)
    set(GSLAM_VERSION_PATCH 0)
  endif(NOT GSLAM_VERSION_PATCH)
	

  set(GSLAM_VERSION ${GSLAM_VERSION_MAJOR}.${GSLAM_VERSION_MINOR}.${GSLAM_VERSION_PATCH})
  set(PROJECT_VERSION ${GSLAM_VERSION})
  set(PROJECT_SOVERSION ${GSLAM_VERSION_MAJOR}.${GSLAM_VERSION_MINOR})
endmacro()


GSLAM_FIND_VERSION()
