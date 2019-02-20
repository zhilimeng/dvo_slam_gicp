set(LIB_NAME dvo_registration)
project(${LIB_NAME})

add_definitions("-DAPI_EXPORTS")
set(REGISTRATION_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/dvo/registration)
set(INCLUDE_FILES
			${REGISTRATION_INCLUDE_DIR}/dense_tracking.h
			${REGISTRATION_INCLUDE_DIR}/dense_tracking_impl.h
			${REGISTRATION_INCLUDE_DIR}/feature_tracking.h
			${REGISTRATION_INCLUDE_DIR}/icp.h 
			${REGISTRATION_INCLUDE_DIR}/point_tracking.h
			)
set(SRC_FILES dense_tracking.cpp
			  dense_tracking_impl.cpp
			  dense_tracking_config.cpp
			  feature_tracking.cpp
			  point_tracking.cpp
			  icp.cpp 
			  )			  
include_directories(${SOPHUS_INCLUDE_DIRS})

add_library(${LIB_NAME} ${DVO_LIB_TYPE} ${INCLUDE_FILES} ${SRC_FILES})
set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries (${LIB_NAME} dvo_core  ${OpenCV_LIBS} ${PCL_LIBRARIES})