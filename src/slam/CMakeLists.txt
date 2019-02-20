set(LIB_NAME dvo_slam)
project(${LIB_NAME})

add_definitions("-DAPI_EXPORTS")
set(SLAM_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/dvo/slam)
set(INCLUDE_FILES
			${SLAM_INCLUDE_DIR}/config.h
			${SLAM_INCLUDE_DIR}/fragment.h
			${SLAM_INCLUDE_DIR}/fragment_optimizer.h
			${SLAM_INCLUDE_DIR}/keyframe_tracker.h
			${SLAM_INCLUDE_DIR}/local_map.h
			${SLAM_INCLUDE_DIR}/local_tracker.h
			)
set(SRC_FILES config.cpp
			  fragment.cpp
			  fragment_optimizer.cpp
			  keyframe_tracker.cpp
			  local_map.cpp 
			  local_tracker.cpp
			  )			  
include_directories(${SOPHUS_INCLUDE_DIRS})
include_directories(${TBB_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
add_library(${LIB_NAME} ${DVO_LIB_TYPE} ${INCLUDE_FILES} ${SRC_FILES})
set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries (${LIB_NAME} dvo_core dvo_registration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBRARIES} ${TBB_IMPORTED_TARGETS})