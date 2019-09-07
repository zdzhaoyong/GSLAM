include(CheckCXXCompilerFlag)
include(CMakeParseArguments)

function(_pybind11_return_if_cxx_and_linker_flags_work result cxxflags linkerflags cxxflags_out linkerflags_out)
  set(CMAKE_REQUIRED_LIBRARIES ${linkerflags})
  check_cxx_compiler_flag("${cxxflags}" ${result})
  if (${result})
    set(${cxxflags_out} "${cxxflags}" CACHE INTERNAL "" FORCE)
    set(${linkerflags_out} "${linkerflags}" CACHE INTERNAL "" FORCE)
  endif()
endfunction()

  if (NOT DEFINED PYBIND11_LTO_CXX_FLAGS)
    set(PYBIND11_LTO_CXX_FLAGS "" CACHE INTERNAL "")
    set(PYBIND11_LTO_LINKER_FLAGS "" CACHE INTERNAL "")

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      set(cxx_append "")
      set(linker_append "")
      if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT APPLE)
        # Clang Gold plugin does not support -Os; append -O3 to MinSizeRel builds to override it
        set(linker_append ";$<$<CONFIG:MinSizeRel>:-O3>")
      elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set(cxx_append ";-fno-fat-lto-objects")
      endif()

      if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND prefer_thin_lto)
        _pybind11_return_if_cxx_and_linker_flags_work(HAS_FLTO_THIN
          "-flto=thin${cxx_append}" "-flto=thin${linker_append}"
          PYBIND11_LTO_CXX_FLAGS PYBIND11_LTO_LINKER_FLAGS)
      endif()

      if (NOT HAS_FLTO_THIN)
        _pybind11_return_if_cxx_and_linker_flags_work(HAS_FLTO
          "-flto${cxx_append}" "-flto${linker_append}"
          PYBIND11_LTO_CXX_FLAGS PYBIND11_LTO_LINKER_FLAGS)
      endif()
    elseif (CMAKE_CXX_COMPILER_ID MATCHES "Intel")
      # Intel equivalent to LTO is called IPO
      _pybind11_return_if_cxx_and_linker_flags_work(HAS_INTEL_IPO
      "-ipo" "-ipo" PYBIND11_LTO_CXX_FLAGS PYBIND11_LTO_LINKER_FLAGS)
    elseif(MSVC)
      # cmake only interprets libraries as linker flags when they start with a - (otherwise it
      # converts /LTCG to \LTCG as if it was a Windows path).  Luckily MSVC supports passing flags
      # with - instead of /, even if it is a bit non-standard:
      _pybind11_return_if_cxx_and_linker_flags_work(HAS_MSVC_GL_LTCG
        "/GL" "-LTCG" PYBIND11_LTO_CXX_FLAGS PYBIND11_LTO_LINKER_FLAGS)
    endif()

    if (PYBIND11_LTO_CXX_FLAGS)
      message(STATUS "LTO enabled")
    else()
      message(STATUS "LTO disabled (not supported by the compiler and/or linker) ${CMAKE_CXX_COMPILER_ID}")
    endif()
  endif()

set(LTO_FOUND TRUE)

list(APPEND LTO_DEFINITIONS -fvisibility=hidden -ffunction-sections -fdata-sections)
foreach(FLAG ${PYBIND11_LTO_CXX_FLAGS})
    list(APPEND LTO_DEFINITIONS ${FLAG})
endforeach()
set(LTO_LIBS "${PYBIND11_LTO_LINKER_FLAGS}")
