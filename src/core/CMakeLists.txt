set(LIB_NAME dvo_core)
project(${LIB_NAME})

add_definitions("-DAPI_EXPORTS")
set(CORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/dvo/core)
set(INCLUDE_FILES
			${CORE_INCLUDE_DIR}/dvo_exports.h
			${CORE_INCLUDE_DIR}/data_types.h 
			${CORE_INCLUDE_DIR}/eigen.h 
			${CORE_INCLUDE_DIR}/frame.h 
			${CORE_INCLUDE_DIR}/hash_eigen.h 
			${CORE_INCLUDE_DIR}/interpolation.h 
			${CORE_INCLUDE_DIR}/intrinsic_matrix.h 
			${CORE_INCLUDE_DIR}/ipoint_cloud.h 
			${CORE_INCLUDE_DIR}/least_squares.h 
			${CORE_INCLUDE_DIR}/math_sse.h 
			${CORE_INCLUDE_DIR}/pointcloud_io.h 
			${CORE_INCLUDE_DIR}/revertable.h 
			${CORE_INCLUDE_DIR}/rgbd_camera.h 
			${CORE_INCLUDE_DIR}/rgbd_image.h 
			${CORE_INCLUDE_DIR}/utility.h 
			${CORE_INCLUDE_DIR}/volume_fusion.h
			${CORE_INCLUDE_DIR}/volume_feature_fusion.h  
			)
set(SRC_FILES 
			 frame.cpp 
			 interpolation.cpp 
			 intrinsic_matrix.cpp 
			 ipoint_cloud.cpp 
			 least_squares.cpp 
			 math_sse.cpp 
			 rgbd_camera.cpp 
			 rgbd_image.cpp 
			 rgbd_image_sse.cpp 
			 utility.cpp 
			 volume_fusion.cpp
			 volume_feature_fusion.cpp)
add_library(${LIB_NAME} ${DVO_LIB_TYPE} ${INCLUDE_FILES} ${SRC_FILES} )
set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries (${LIB_NAME}  ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})