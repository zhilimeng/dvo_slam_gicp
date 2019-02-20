####
# Find G2O
# This sets the following variables:
# G2O_FOUND - True if G2O was found
# G2O_INCLUDE_DIRS - Directories containing the G2O include files
# G2O_LIBRARIES - Libraries needed to use G2O.

# find g2o 
find_path(G2O_INCLUDE_DIR g2o/config.h 
    HINTS "${G2O_ROOT}" "$ENV{G2O_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/g2o" "$ENV{PROGRAMW6432}/g2o"
		  "${CMAKE_SOURCE_DIR}/3rdParty/g2o_2017_0730/include"
    PATH_SUFFIXES g2o include/g2o include
	NO_DEFAULT_PATH)
find_path(G2O_EXT_INCLUDE_DIR "cs.h"
	HINTS "${G2O_ROOT}" "$ENV{G2O_ROOT}"
	PATHS "$ENV{PROGRAMFILES}/g2o" "$ENV{PROGRAMW6432}/g2o"
		  "${G2O_INCLUDE_DIR}/EXTERNAL/csparse"
	PATH_SUFFIXES csparse EXTERNAL/csparse
	NO_DEFAULT_PATH)
set(G2O_INCLUDE_DIR "${G2O_INCLUDE_DIR}" CACHE PATH "g2o include dir." FORCE)
set(G2O_ROOT "${G2O_INCLUDE_DIR}/../")
set(G2O_REQUIRED_COMPONENT 	 ext_csparse stuff core cli  solver_cholmod solver_csparse csparse_extension solver_dense solver_pcg
							solver_slam2d_linear solver_structure_only types_data types_icp types_sba types_sclam2d
							types_sim3 types_slam2d types_slam3d)
foreach(module ${G2O_REQUIRED_COMPONENT})
  set(g2o_component g2o_${module})
  string(TOUPPER "${g2o_component}" COMPONENT)
  find_library("${COMPONENT}_RELEASE"
				NAMES "${g2o_component}"
				HINTS "ENV{G2O_ROOT}"
				PATHS "ENV{PROGRAMFILES}/g2o" "ENV{PROGRAMW6432}/g2o"
					"${G2O_ROOT}/lib"
				PATH_SUFFIXES 
				NO_DEFAULT_PATH)
  find_library("${COMPONENT}_DEBUG"
				NAMES "${g2o_component}_d"
				HINTS "ENV{G2O_ROOT}"
				PATHS "ENV{PROGRAMFILES}/g2o" "ENV{PROGRAMW6432}/g2o"
					"${G2O_ROOT}/lib"
				PATH_SUFFIXES 
				NO_DEFAULT_PATH)
  if(NOT ${COMPONENT}_DEBUG)
    if(${COMPONENT}_RELEASE)
	  set(${COMPONENT}_DEBUG ${${COMPONENT}_RELEASE})
	endif(${COMPONENT}_RELEASE)
  endif(NOT ${COMPONENT}_DEBUG)

  list(APPEND G2O_LIBRARY ${${COMPONENT}_RELEASE})
  list(APPEND G2O_LIBRARY_DEBUG ${${COMPONENT}_DEBUG})
  set(G2O_${COMPONENT}_LIBRARIES optimized ${${COMPONENT}_RELEASE} debug ${${COMPONENT}_DEBUG})
  list(APPEND G2O_LIBRARIES ${G2O_${COMPONENT}_LIBRARIES})
endforeach(module)

set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR} ${G2O_EXT_INCLUDE_DIR})
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O DEFAULT_MSG G2O_LIBRARY G2O_INCLUDE_DIRS)