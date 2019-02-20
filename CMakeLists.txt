cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(DVO)

# the version number
set(DVO_VERSION_MAJOR 1)
set(DVO_VERSION_MINOR 0)

# cmake configuration type
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)
# In case the user does not setup CMAKE_BUILD_TYPE, assume it's Release
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "build type default to Release, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/ ${CMAKE_MODULE_PATH})


# ---[ Release/Debug specific flags
if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "_debug")
  endif()
  if(NOT DEFINED CMAKE_RELEASE_POSTFIX)
    set(CMAKE_RELEASE_POSTFIX "_release")
  endif()
  if(NOT DEFINED CMAKE_RELWITHDEBINFO_POSTFIX)
    set(CMAKE_RELWITHDEBINFO_POSTFIX "_release")
  endif()
  if(NOT DEFINED CMAKE_MINSIZEREL_POSTFIX)
    set(CMAKE_MINSIZEREL_POSTFIX "_release")
  endif()
endif()

# build shared libraries by default
option(BUILD_SHARED_LIBS "build shared libraries." ON)
if(BUILD_SHARED_LIBS)
	set(DVO_LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
	set(DVO_LIB_SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
	set(DVO_LIB_TYPE "SHARED")
	if(WIN32)
		set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_IMPORT_LIBRARY_SUFFIX})
	endif(WIN32)
else(BUILD_SHARED_LIBS)
	set(DVO_LIB_PREFIX ${CMAKE_STATIC_LIBRARY_PREFIX})
	set(DVO_LIB_SUFFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
	set(DVO_LIB_TYPE "STATIC")
	set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
endif(BUILD_SHARED_LIBS)

# set lib/dll/exe output dir
set(DVO_OUTPUT_LIB_DIR "${PROJECT_BINARY_DIR}/lib")
set(DVO_OUTPUT_BIN_DIR "${PROJECT_BINARY_DIR}/bin")

if(WIN32)
	foreach(config ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${config} CONFIG)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} "${DVO_OUTPUT_LIB_DIR}/${config}")
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} "${DVO_OUTPUT_BIN_DIR}/${config}")
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} "${DVO_OUTPUT_BIN_DIR}/${config}")
	endforeach(config)
else(WIN32)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${DVO_OUTPUT_LIB_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${DVO_OUTPUT_BIN_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${DVO_OUTPUT_BIN_DIR}")
endif(WIN32)

if(WIN32)
	add_definitions("-D_USE_MATH_DEFINES")
endif(WIN32)

option(BUILD_EXAMPLES "build exmaples or not " ON)

# find OpenCV
find_package(OpenCV REQUIRED)

# find PCL // also find Eigen Boost FLANN Qhull VTK
find_package(PCL REQUIRED)

# find Sophus 
find_package(Sophus REQUIRED)

# find G2O
find_package(G2O REQUIRED)

# find TBB
find_package(TBB REQUIRED)
#include(${TBB_DIR}/TBBConfig.cmake)
message(STATUS "TBB component: ${TBB_IMPORTED_TARGETS}")
# set lib link directories
link_directories(${PROJECT_SOURCE_DIR}/lib)

# set include directories
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_BINARY_DIR})

# add subdirectory
add_subdirectory(${CMAKE_SOURCE_DIR}/src)
if(BUILD_EXAMPLES)
	add_subdirectory(${CMAKE_SOURCE_DIR}/examples)
endif(BUILD_EXAMPLES)