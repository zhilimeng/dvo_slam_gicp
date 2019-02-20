#########################################
####
# Find Sophus
# This sets the following variables:
# SOPHUS_FOUND - True if Sophus was found
# SOPHUS_INCLUDE_DIRS - Directories containing the Sophus include files

find_path(SOPHUS_INCLUDE_DIR sophus/se3.hpp
    HINTS "${SOPHUS_ROOT}" "$ENV{SOPHUS_ROOT}" "${SOPHUS_INCLUDE_DIR}"
    PATHS "$ENV{PROGRAMFILES}/Sophus" "$ENV{PROGRAMW6432}/Sophus"
		  "${CMAKE_SOURCE_DIR}/3rdParty/sophus_1_00/include"
    PATH_SUFFIXES sophus include/sophus include)
	
set(SOPHUS_INCLUDE_DIR "${SOPHUS_INCLUDE_DIR}" CACHE PATH "Sophus include dir." FORCE)
set(SOPHUS_ROOT "${CMAKE_SOURCE_DIR}/3rdParty/sophus_1_00")


set(SOPHUS_INCLUDE_DIRS ${SOPHUS_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SOPHUS DEFAULT_MSG SOPHUS_INCLUDE_DIR)
if(SOPHUS_FOUND)
	set(HAVE_SOPHUS ON)
	message(STATUS "SOPHUS found (include:${SOPHUS_INCLUDE_DIRS})")
endif(SOPHUS_FOUND)
					