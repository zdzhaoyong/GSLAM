######################################################################################
# PICMake VERSION 1.1.2
# HISTORY:
#   1.0.0 2017.01.04 : first commit, one include for one target.
#   1.1.0 2017.01.09 : support multi targets, reorgnized functions and macros.
#   1.1.1 2017.01.09 : added pi_install with install and uninstall support.
#   1.1.2 2017.01.10 : added pi_parse_arguments to support cmake version less than 3.1 
#                      modified pi_install&pi_collect_packages&pi_add_target
#                      fixed bug of failed make uninstall
######################################################################################
#                               FUNCTIONS
# pi_collect_packagenames(<RESULT_NAME>　[VERBOSE] [path1 path2 ...])
# pi_removesource(<VAR_NAME> <regex>)
# pi_hasmainfunc(<RESULT_NAME> source1 [source2 ...])
# pi_add_target(<name> <BIN | STATIC | SHARED> [src1|dir1 ...] [MODULES module1 ...] [REQUIRED module1 ...] [DEPENDENCY target1 ...])
# pi_add_targets([name1 ...])
# pi_report_target([LIBS2COMPILE] [APPS2COMPILE])
# pi_install([HEADERS header1|dir1 ...] [TARGETS target1 ...] [CMAKE cmake_config] [BIN_DESTINATION dir] [LIB_DESTINATION dir] [HEADER_DESTINATION dir])
# pi_parse_arguments(<prefix> <options> <one_value_keywords> <multi_value_keywords> args...)
######################################################################################
#                               MACROS
# pi_collect_packages(<RESULT_NAME> [VERBOSE] [MODULES package1 ...] [REQUIRED package1 package2 ...])
# pi_check_modules(module1 [module2 ...])
# pi_report_modules(module1 [module2 ...])
######################################################################################

#cmake_minimum_required(VERSION 2.6)
if(PICMAKE_LOADED)
  return()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(PICMAKE_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(PI_CMAKE_VERSION_MAJOR 1)
set(PI_CMAKE_VERSION_MINOR 1)
set(PI_CMAKE_VERSION_PATCH 2)
set(PI_CMAKE_VERSION "${PI_CMAKE_VERSION_MAJOR}.${PI_CMAKE_VERSION_MINOR}.${PI_CMAKE_VERSION_PATCH}")

# pi_collect_packagenames(<RESULT_NAME> [path1 path2 ...])
function(pi_collect_packagenames RESULT_NAME)
  if(NOT ARGN)
    list(APPEND COLLECT_PATHS ${CMAKE_MODULE_PATH} ${CMAKE_ROOT}/Modules)
  else()
    set(COLLECT_PATHS ${ARGN})  
  endif()
  
  message("COLLECT_PATHS: ${COLLECT_PATHS}")

  foreach(COLLECT_PATH ${COLLECT_PATHS})
    file(GLOB PACKAGES RELATIVE ${COLLECT_PATH} ${COLLECT_PATH}/Find*.cmake)
    #message("PACKAGES: ${PACKAGES}")
    foreach(PACKAGE_PATH ${PACKAGES})
      string(REGEX MATCH "Find([a-z|A-Z|0-9]+).cmake" PACKAGE_NAME "${PACKAGE_PATH}")
      set(PACKAGE_NAME "${CMAKE_MATCH_1}")
      list(APPEND COLLECT_PACKAGE_NAMES ${PACKAGE_NAME})
      #message("PACKAGE_PATH: ${PACKAGE_PATH}")
      #message("PACKAGE_NAME: ${PACKAGE_NAME}")
    endforeach()
  endforeach()

  list(REMOVE_DUPLICATES COLLECT_PACKAGE_NAMES)
  list(SORT COLLECT_PACKAGE_NAMES)
  set(${RESULT_NAME} ${COLLECT_PACKAGE_NAMES} PARENT_SCOPE)
endfunction()

# pi_removesource(<VAR_NAME> <regex>)
function(pi_removesource VAR_NAME regex)
  foreach(SRC_FILE ${${VAR_NAME}})
    string(REGEX MATCH "${regex}" SHOULDREMOVE ${SRC_FILE})
    if(SHOULDREMOVE)
      list(REMOVE_ITEM ${VAR_NAME} ${SRC_FILE})
    endif()
  endforeach()
  set(${VAR_NAME} ${${VAR_NAME}} PARENT_SCOPE)
endfunction()

# pi_hasmainfunc(<RESULT_NAME> source1 [source2 ...])
function(pi_hasmainfunc RESULT_NAME)
  foreach(SOURCE_FILE ${ARGN})
    get_filename_component(SRC_FILE_NAME ${SOURCE_FILE} NAME_WE)
    string(TOLOWER ${SRC_FILE_NAME} SRC_FILE_NAME)
    if(SRC_FILE_NAME STREQUAL "main")
      list(APPEND MAIN_FILES ${SOURCE_FILE})
    endif()
  endforeach()
  set(${RESULT_NAME} ${MAIN_FILES} PARENT_SCOPE)
endfunction()

# pi_add_target(<name> <BIN | STATIC | SHARED> [src1|dir1 ...] [MODULES module1 ...] [REQUIRED module1 ...] [DEPENDENCY target1 ...])
function(pi_add_target TARGET_NAME TARGET_TYPE)
  if(ARGC LESS 2)
    message("command 'add_target' need more than 2 arguments")
    return()
  endif(ARGC LESS 2)

  string(TOUPPER ${TARGET_TYPE} TARGET_TYPE)
  
  pi_parse_arguments(PI_TARGET "" "" "MODULES;REQUIRED;DEPENDENCY" ${ARGN})
  set(TARGET_MODULES ${PI_TARGET_MODULES})
  set(TARGET_REQUIRED ${PI_TARGET_REQUIRED})
  set(TARGET_DEPENDENCY ${PI_TARGET_DEPENDENCY})

  set(TARGET_COMPILEFLAGS)
  set(TARGET_LINKFLAGS)
  set(TARGET_DEFINITIONS)
  
  if(NOT PI_TARGET_UNPARSED_ARGUMENTS)
    set(PI_TARGET_UNPARSED_ARGUMENTS ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  set(TARGET_SRCS )
  foreach(PI_TARGET_SRC ${PI_TARGET_UNPARSED_ARGUMENTS})
      get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${PI_TARGET_SRC}" ABSOLUTE)
      if(IS_DIRECTORY ${ABSOLUTE_PATH})
        file(GLOB_RECURSE PATH_SOURCE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${PI_TARGET_SRC}/*.cpp ${PI_TARGET_SRC}/*.c ${PI_TARGET_SRC}/*.cc])
        list(APPEND TARGET_SRCS ${PATH_SOURCE_FILES})
      else()
        list(APPEND TARGET_SRCS ${PI_TARGET_SRC})
      endif()
  endforeach()

  if(NOT TARGET_SRCS)
    message("add_target(${ARGV}) need at least 1 source file.")
    return()
  endif()

  if( NOT ((TARGET_TYPE STREQUAL "BIN") OR (TARGET_TYPE STREQUAL "STATIC") OR (TARGET_TYPE STREQUAL "SHARED")))
    message("TARGET_TYPE (${TARGET_TYPE}) should be BIN STATIC or SHARED")
  endif()

  foreach(MODULE_NAME ${TARGET_REQUIRED})
    string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)
    if(TARGET ${MODULE_NAME})
      #message("${MODULE_NAME} is a existed target")
      list(APPEND TARGET_LINKFLAGS ${MODULE_NAME})
    elseif(${MODULE_NAME_UPPER}_FOUND)
      list(APPEND TARGET_COMPILEFLAGS ${${MODULE_NAME_UPPER}_INCLUDES})
      list(APPEND TARGET_LINKFLAGS ${${MODULE_NAME_UPPER}_LIBRARIES})
      list(APPEND TARGET_DEFINITIONS ${${MODULE_NAME_UPPER}_DEFINITIONS})
    else()
      message("${TARGET_NAME} aborded since can't find dependency ${MODULE_NAME}.")
      return()
    endif()
  endforeach()
  
  foreach(MODULE_NAME ${TARGET_MODULES})
    string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)
    if(TARGET ${MODULE_NAME})
      #message("${MODULE_NAME} is a existed target")
      list(APPEND TARGET_LINKFLAGS ${MODULE_NAME})
    elseif(${MODULE_NAME_UPPER}_FOUND)
      list(APPEND TARGET_COMPILEFLAGS ${${MODULE_NAME_UPPER}_INCLUDES})
      list(APPEND TARGET_LINKFLAGS ${${MODULE_NAME_UPPER}_LIBRARIES})
      list(APPEND TARGET_DEFINITIONS ${${MODULE_NAME_UPPER}_DEFINITIONS})
    endif()
  endforeach()

  include_directories(${TARGET_COMPILEFLAGS})
  add_definitions(${TARGET_DEFINITIONS})

  if(TARGET_TYPE STREQUAL "BIN")
    set_property( GLOBAL APPEND PROPERTY APPS2COMPILE  " ${TARGET_NAME}")
    add_executable(${TARGET_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${TARGET_SRCS})
  elseif(TARGET_TYPE STREQUAL "STATIC")
    set_property( GLOBAL APPEND PROPERTY LIBS2COMPILE  " ${CMAKE_STATIC_LIBRARY_PREFIX}${TARGET_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX}")
    add_library(${TARGET_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR} ${TARGET_SRCS})
  elseif(TARGET_TYPE STREQUAL "SHARED")
    set_property( GLOBAL APPEND PROPERTY LIBS2COMPILE  " ${CMAKE_SHARED_LIBRARY_PREFIX}${TARGET_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")
    add_library(${TARGET_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR} ${TARGET_SRCS})
    if(PROJECT_VERSION)
      set_target_properties(${TARGET_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
    endif()
    if(PROJECT_SOVERSION)
      set_target_properties(${TARGET_NAME} PROPERTIES SOVERSION ${PROJECT_SOVERSION})      
    endif()
    #message("add_library(${TARGET_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR} ${TARGET_SRCS})")
  else()
    message("add_target(TARGET_TYPE ${TARGET_TYPE}): THIS SHOULD NEVER HAPPEN!")
    return()
  endif()

  #message("ARGV: ${ARGV}")
  #message("TARGET_NAME: ${TARGET_NAME}")
  #message("TARGET_SRCS: ${TARGET_SRCS}")
  #message("TARGET_TYPE: ${TARGET_TYPE}")
  #message("TARGET_MODULES: ${TARGET_MODULES}")
  #message("TARGET_REQUIRED: ${TARGET_REQUIRED}")
  #message("TARGET_COMPILEFLAGS: ${TARGET_COMPILEFLAGS}")

  target_link_libraries(${TARGET_NAME} ${TARGET_LINKFLAGS} ${TARGET_DEPENDENCY})
  list(APPEND TARGET_MODULES ${TARGET_REQUIRED})
  if("${TARGET_MODULES}" MATCHES "Qt|QT|qt")
      #message("Compile ${TARGET_NAME} with AUTOMOC (${TARGET_MODULES})")
      set_target_properties(${TARGET_NAME} PROPERTIES AUTOMOC TRUE)
  endif()

endfunction(pi_add_target)

# pi_add_targets([name1 ...])
# TARGET_NAME     -- TARGET_NAME  -- Folder name
# TARGET_SRCS     -- TARGET_SRCS  -- All source files below ${CMAKE_CURRENT_SOURCE_DIR}
# TARGET_TYPE     -- TARGET_TYPE|MAKE_TYPE    -- BIN STATIC SHARED
# TARGET_MODULES  -- TARGET_MODULES|MODULES      -- All packages available
# TARGET_REQUIRED -- TARGET_REQUIRED|REQUIRED
function(pi_add_targets )
  if(ARGC LESS 2)
    if(ARGC EQUAL 1)
      set(TARGET_NAME ${ARGV})
      #message("TARGET_NAME: ${TARGET_NAME}")
      if(TARGET_NAME STREQUAL "NO_TARGET")
        return()
      endif()
    elseif(NOT TARGET_NAME)
            get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
            string(REPLACE " " "_" TARGET_NAME ${TARGET_NAME})
            #message("Use folder name target ${TARGET_NAME}")
    endif()

    if(NOT TARGET_SRCS)
      set(TARGET_SRCS ${SOURCE_FILES_ALL})
    endif()

    if(NOT TARGET_SRCS)
      file(GLOB_RECURSE TARGET_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.c *.cc])
      #pi_removesource(TARGET_SRCS "CMakeFiles")
    endif()

    if(NOT TARGET_TYPE)
      set(TARGET_TYPE ${MAKE_TYPE})
    endif()

    if(NOT TARGET_TYPE)
      pi_hasmainfunc(MAIN_FILES ${TARGET_SRCS})
      if(MAIN_FILES)
        set(TARGET_TYPE BIN)
      elseif(BUILD_SHARED_LIBS)
        set(TARGET_TYPE SHARED)
      else()
        set(TARGET_TYPE STATIC)
      endif(MAIN_FILES)
    endif()

    if(NOT TARGET_MODULES)
      set(TARGET_MODULES ${MODULES})
    endif()
    #message("TARGET_TYPE: ${TARGET_TYPE}")

    pi_add_target(${TARGET_NAME} ${TARGET_TYPE} ${TARGET_SRCS} MODULES ${TARGET_MODULES} REQUIRED ${TARGET_REQUIRED})
    return()
  endif(ARGC LESS 2)

  foreach(TARGET_NAME ${ARGV})
    if(${TARGET_NAME}_SRCS)
      if(NOT ${TARGET_NAME}_TYPE)
        pi_hasmainfunc(MAIN_FILES ${${TARGET_NAME}_SRCS})
        if(MAIN_FILES)
          set(${TARGET_NAME}_TYPE BIN)
        else()
          set(${TARGET_NAME}_TYPE SHARED)
        endif()
        pi_add_target(${TARGET_NAME} ${${TARGET_NAME}_TYPE} ${${TARGET_NAME}_SRCS} MODULES ${${TARGET_NAME}_MODULES} REQUIRED ${${TARGET_NAME}_REQUIRED})
      endif()
    else()
      message("Target ${TARGET_NAME} aborded since no source file found.")
    endif()
  endforeach()


endfunction(pi_add_targets)


# pi_report_target([LIBS2COMPILE] [APPS2COMPILE])
function(pi_report_target )
  get_property(LIBS2COMPILE GLOBAL PROPERTY LIBS2COMPILE)
  get_property(APPS2COMPILE GLOBAL PROPERTY APPS2COMPILE)

  message(STATUS "The following targets will to be build:")
  message(STATUS "LIBS(${CMAKE_LIBRARY_OUTPUT_DIRECTORY}): ${LIBS2COMPILE}")
  message(STATUS "APPS(${CMAKE_RUNTIME_OUTPUT_DIRECTORY}): ${APPS2COMPILE}")

  set(INDEX 1)
  if(ARGV${INDEX})
    set(${ARGV${INDEX}} ${LIBS2COMPILE} PARENT_SCOPE)
  endif()
  set(INDEX 2)
  if(ARGV${INDEX})
    set(${ARGV${INDEX}} ${APPS2COMPILE} PARENT_SCOPE)
  endif()
endfunction()

function(pi_report_targets)
  pi_report_target(${ARGV})
endfunction()


# pi_install([HEADERS header1|dir1 ...] [TARGETS target1 ...] [CMAKE cmake_config] [BIN_DESTINATION dir] [LIB_DESTINATION dir] [HEADER_DESTINATION dir])
function(pi_install)
  pi_parse_arguments(PI_INSTALL "VERBOSE" "BIN_DESTINATION;LIB_DESTINATION;HEADER_DESTINATION" "HEADERS;TARGETS;CMAKE" ${ARGN})

  if(PI_INSTALL_VERBOSE)
    message("PI_INSTALL_BIN_DESTINATION: ${PI_INSTALL_BIN_DESTINATION}")
    message("PI_INSTALL_LIB_DESTINATION: ${PI_INSTALL_LIB_DESTINATION}")
    message("PI_INSTALL_HEADER_DESTINATION: ${PI_INSTALL_HEADER_DESTINATION}")
    message("PI_INSTALL_HEADERS: ${PI_INSTALL_HEADERS}")
    message("PI_INSTALL_TARGETS: ${PI_INSTALL_TARGETS}")
    message("PI_INSTALL_CMAKE: ${PI_INSTALL_CMAKE}")
  endif()
  
  if(NOT PI_INSTALL_BIN_DESTINATION)
    set(PI_INSTALL_BIN_DESTINATION bin)
  endif()
  
  if(NOT PI_INSTALL_LIB_DESTINATION)
    set(PI_INSTALL_LIB_DESTINATION lib)
  endif()

  if(NOT PI_INSTALL_HEADER_DESTINATION)
    set(PI_INSTALL_HEADER_DESTINATION include)
  endif()

  foreach(INSTALL_HEADER ${PI_INSTALL_HEADERS})
    #message("get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${INSTALL_HEADER}" ABSOLUTE)")
    get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${INSTALL_HEADER}" ABSOLUTE)
    if(IS_DIRECTORY ${ABSOLUTE_PATH})
      #message("install(DIRECTORY ${INSTALL_HEADER} DESTINATION "${HEADER_DESTINATION}" FILES_MATCHING PATTERN "*.h")")
      install(DIRECTORY ${ABSOLUTE_PATH} DESTINATION "${PI_INSTALL_HEADER_DESTINATION}" FILES_MATCHING PATTERN "*.h")
      install(DIRECTORY ${ABSOLUTE_PATH} DESTINATION "${PI_INSTALL_HEADER_DESTINATION}" FILES_MATCHING PATTERN "*.hpp")
    else()
      #message("install(FILES ${INSTALL_HEARDERS} DESTINATION ${HEADER_DESTINATION} COMPONENT main) ")
      install(FILES ${INSTALL_HEARDERS} DESTINATION ${PI_INSTALL_HEADER_DESTINATION} COMPONENT main)      
    endif()
  endforeach()

  foreach(TARGET ${PI_INSTALL_TARGETS})
    if(TARGET ${TARGET})
      install(TARGETS ${TARGET}
          RUNTIME DESTINATION ${PI_INSTALL_BIN_DESTINATION} COMPONENT main
          LIBRARY DESTINATION ${PI_INSTALL_LIB_DESTINATION} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
          ARCHIVE DESTINATION ${PI_INSTALL_LIB_DESTINATION} COMPONENT main)
    endif()
  endforeach()

  foreach(CONFG_FILE ${PI_INSTALL_CMAKE})
    get_filename_component(CONFIG_NAME "${CONFG_FILE}" NAME_WE)
    configure_file("${CONFG_FILE}" "${PROJECT_BINARY_DIR}/${CONFIG_NAME}.cmake" @ONLY)
    install(FILES "${PROJECT_BINARY_DIR}/${CONFIG_NAME}.cmake" DESTINATION ${CMAKE_ROOT}/Modules)
  endforeach()
  

# Auto uninstall
  if(NOT EXISTS "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")
    file(WRITE "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake" "IF(NOT EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\")
  MESSAGE(FATAL_ERROR \"Cannot find install manifest: ${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\")
ENDIF()
FILE(READ \"${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\" files)
STRING(REGEX REPLACE \"\\n\" \";\" files \"\${files}\")
FOREACH(file \${files})
  MESSAGE(STATUS \"Uninstalling \${file}\")
  #EXECUTE_PROCESS(COMMAND rm \"\${file}\")
  file(REMOVE \"\${file}\")
ENDFOREACH()
    ")
  else()
    add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")
  endif()

endfunction()

# pi_parse_arguments(<prefix> <options> <one_value_keywords> <multi_value_keywords> args...)
function(pi_parse_arguments prefix _optionNames _singleArgNames _multiArgNames)
  # first set all result variables to empty/FALSE
  foreach(arg_name ${_singleArgNames} ${_multiArgNames})
    set(${prefix}_${arg_name})
  endforeach()

  foreach(option ${_optionNames})
    set(${prefix}_${option} FALSE)
  endforeach()

  set(${prefix}_UNPARSED_ARGUMENTS)

  set(insideValues FALSE)
  set(currentArgName)

  # now iterate over all arguments and fill the result variables
  foreach(currentArg ${ARGN})
    list(FIND _optionNames "${currentArg}" optionIndex)  # ... then this marks the end of the arguments belonging to this keyword
    list(FIND _singleArgNames "${currentArg}" singleArgIndex)  # ... then this marks the end of the arguments belonging to this keyword
    list(FIND _multiArgNames "${currentArg}" multiArgIndex)  # ... then this marks the end of the arguments belonging to this keyword

    if(${optionIndex} EQUAL -1  AND  ${singleArgIndex} EQUAL -1  AND  ${multiArgIndex} EQUAL -1)
      if(insideValues)
        if("${insideValues}" STREQUAL "SINGLE")
          set(${prefix}_${currentArgName} ${currentArg})
          set(insideValues FALSE)
        elseif("${insideValues}" STREQUAL "MULTI")
          list(APPEND ${prefix}_${currentArgName} ${currentArg})
        endif()
      else()
        list(APPEND ${prefix}_UNPARSED_ARGUMENTS ${currentArg})
      endif()
    else()
      if(NOT ${optionIndex} EQUAL -1)
        set(${prefix}_${currentArg} TRUE)
        set(insideValues FALSE)
      elseif(NOT ${singleArgIndex} EQUAL -1)
        set(currentArgName ${currentArg})
        set(${prefix}_${currentArgName})
        set(insideValues "SINGLE")
      elseif(NOT ${multiArgIndex} EQUAL -1)
        set(currentArgName ${currentArg})
        set(${prefix}_${currentArgName})
        set(insideValues "MULTI")
      endif()
    endif()

  endforeach()

  # propagate the result variables to the caller:
  foreach(arg_name ${_singleArgNames} ${_multiArgNames} ${_optionNames})
    set(${prefix}_${arg_name}  ${${prefix}_${arg_name}} PARENT_SCOPE)
  endforeach()
  set(${prefix}_UNPARSED_ARGUMENTS ${${prefix}_UNPARSED_ARGUMENTS} PARENT_SCOPE)

endfunction()
######################################################################################
#                               MACROS

# pi_collect_packages([RESULT_NAME] [VERBOSE] [MODULES package1 ...] [REQUIRED package1 package2 ...])
macro(pi_collect_packages)
  pi_parse_arguments(PI_COLLECT "VERBOSE" "" "MODULES;REQUIRED" ${ARGV})
  list(LENGTH PI_COLLECT_UNPARSED_ARGUMENTS PI_COLLECT_UNPARSED_NUM)
  if(PI_COLLECT_UNPARSED_ARGUMENTS GREATER 1)
    message("Error parsing pi_collect_packages(${ARGV})")
    return()
  else()
    set(RESULT_NAME ${PI_COLLECT_UNPARSED_ARGUMENTS})
  endif()

  if( (NOT PI_COLLECT_MODULES) AND (NOT PI_COLLECT_REQUIRED) )
    pi_collect_packagenames(PI_COLLECT_MODULES)
  endif()

  if(NOT PI_COLLECT_VERBOSE)
    set(PI_COLLECT_FLAGS QUIET)
  endif()

  foreach(PACKAGE_NAME ${PI_COLLECT_REQUIRED})
    find_package(${PACKAGE_NAME} REQUIRED ${PI_COLLECT_FLAGS})
  endforeach()
  
  foreach(PACKAGE_NAME ${PI_COLLECT_MODULES})
    find_package(${PACKAGE_NAME} ${PI_COLLECT_FLAGS})
  endforeach()

  list(APPEND PI_COLLECT_MODULES ${PI_COLLECT_REQUIRED})

  if(PI_COLLECT_VERBOSE)
    pi_report_modules(${PI_COLLECT_MODULES})
  else()
    pi_check_modules(${PI_COLLECT_REQUIRED})
  endif()

  foreach(PACKAGE_NAME ${PI_COLLECT_MODULES})
    string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UPPER)
    if(${PACKAGE_NAME_UPPER}_FOUND)
      list(APPEND ${RESULT_NAME} ${PACKAGE_NAME})
    endif()
  endforeach()
  
endmacro()

# pi_check_module_part(<module_name> <part_source> <part_target>)
macro(pi_check_module_part MODULE_NAME PART_SOURCE PART_TARGET)
  string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)
  if(NOT ${MODULE_NAME_UPPER}_${PART_TARGET})
    if(${MODULE_NAME}_${PART_SOURCE})
      set(${MODULE_NAME_UPPER}_${PART_TARGET} ${${MODULE_NAME}_${PART_SOURCE}})
    elseif(${MODULE_NAME}_${PART_TARGET})
      set(${MODULE_NAME_UPPER}_${PART_TARGET} ${${MODULE_NAME}_${PART_TARGET}})
    elseif(${MODULE_NAME_UPPER}_${PART_SOURCE})
      set(${MODULE_NAME_UPPER}_${PART_TARGET} ${${MODULE_NAME_UPPER}_${PART_SOURCE}})
    endif()
  endif()
endmacro()

# pi_check_modules(module1 [module2 ...])
macro(pi_check_modules)
  foreach(MODULE_NAME ${ARGV})
    pi_check_module_part(${MODULE_NAME} INCLUDE_DIR INCLUDES)
    pi_check_module_part(${MODULE_NAME} LIBS LIBRARIES)
    pi_check_module_part(${MODULE_NAME} LIBRARY LIBRARIES)
    pi_check_module_part(${MODULE_NAME} found FOUND)
    pi_check_module_part(${MODULE_NAME} version VERSION)
    pi_check_module_part(${MODULE_NAME} DEFINITIONS DEFINITIONS)
    if(${MODULE_NAME_UPPER}_FOUND)
      list(APPEND ${MODULE_NAME_UPPER}_DEFINITIONS -DHAS_${MODULE_NAME_UPPER})
      list(REMOVE_DUPLICATES ${MODULE_NAME_UPPER}_DEFINITIONS)
    endif()    
  endforeach()
endmacro()


#pi_report_modules(module1 [module2 ...])
macro(pi_report_modules)
  pi_check_modules(${ARGV})
  foreach(MODULE_NAME ${ARGV})
    message("--------------------------------------")
    string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPER)
    if(${MODULE_NAME_UPPER}_VERSION)
      message("--${MODULE_NAME}: VERSION ${${MODULE_NAME_UPPER}_VERSION}")
    else()
      message("--${MODULE_NAME}:")
    endif()

    if(${MODULE_NAME_UPPER}_INCLUDES)
      message("  ${MODULE_NAME_UPPER}_INCLUDES: ${${MODULE_NAME_UPPER}_INCLUDES}")
    endif()


    if(${MODULE_NAME_UPPER}_LIBRARIES)
      message("  ${MODULE_NAME_UPPER}_LIBRARIES: ${${MODULE_NAME_UPPER}_LIBRARIES}")
    endif()

    if(${MODULE_NAME_UPPER}_DEFINITIONS)
      message("  ${MODULE_NAME_UPPER}_DEFINITIONS: ${${MODULE_NAME_UPPER}_DEFINITIONS}")
    endif()

  endforeach()
endmacro()


##########################################################
#           THINGS GOING TO REMOVE! DO NOT USE!
macro(autosetMakeType)
  if(BUILD_SHARED_LIBS)
    set(MAKE_TYPE "shared")
  else()
    set(MAKE_TYPE "static")
  endif()

  pi_hasmainfunc(MAIN_FILES ${SOURCE_FILES_ALL})
  
  if(MAIN_FILES)
    set(MAKE_TYPE  "bin")
  endif()
endmacro()

# Results collection
macro(PIL_CHECK_DEPENDENCY LIBNAME)
  pi_check_modules(${LIBNAME})
endmacro()

macro(PIL_ECHO_LIBINFO LIBNAME)
  pi_report_modules(${LIBNAME})
endmacro()

# The following things is deprected
macro(filtSOURCE_FILES_ALL DIR)
  pi_removesource(SOURCE_FILES_ALL ${DIR})
endmacro()

macro(reportTargets)
  pi_report_target()
endmacro()


set(PICMAKE_UTILS_LOADED TRUE)
set(PICMAKE_LOADED TRUE)
