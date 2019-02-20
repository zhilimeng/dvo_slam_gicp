set(LIB_NAME dvo_surface)
project(${LIB_NAME})

add_definitions("-DAPI_EXPORTS")
set(SURFACE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/dvo/surface)
set(POISSON_INCLUDES
	${SURFACE_INCLUDE_DIR}/poisson8/allocator.h 
	${SURFACE_INCLUDE_DIR}/poisson8/array.h
	${SURFACE_INCLUDE_DIR}/poisson8/array.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/binary_node.h
	${SURFACE_INCLUDE_DIR}/poisson8/bspline_data.h 
	${SURFACE_INCLUDE_DIR}/poisson8/bspline_data.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/geometry.h 
	${SURFACE_INCLUDE_DIR}/poisson8/geometry.hpp 
	${SURFACE_INCLUDE_DIR}/poisson8/hash.h 
	${SURFACE_INCLUDE_DIR}/poisson8/marching_cubes.h
	${SURFACE_INCLUDE_DIR}/poisson8/mat.h
	${SURFACE_INCLUDE_DIR}/poisson8/mat.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data.h
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data_evaluation.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data_isosurface.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data_sorted_tree_nodes.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data_system.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/multi_grid_octree_data_weighted_samples.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/octree.h 
	${SURFACE_INCLUDE_DIR}/poisson8/octree.hpp 
	${SURFACE_INCLUDE_DIR}/poisson8/ply.h 
	${SURFACE_INCLUDE_DIR}/poisson8/point_stream.h 
	${SURFACE_INCLUDE_DIR}/poisson8/point_stream.hpp
	${SURFACE_INCLUDE_DIR}/poisson8/polynomial.h 
	${SURFACE_INCLUDE_DIR}/poisson8/polynomial.hpp 
	${SURFACE_INCLUDE_DIR}/poisson8/ppolynomial.h 
	${SURFACE_INCLUDE_DIR}/poisson8/ppolynomial.hpp 
	${SURFACE_INCLUDE_DIR}/poisson8/sparse_matrix.h 
	${SURFACE_INCLUDE_DIR}/poisson8/sparse_matrix.hpp 
	${SURFACE_INCLUDE_DIR}/poisson8/surface_trimmer.h 
	${SURFACE_INCLUDE_DIR}/poisson8/surface_trimmer.hpp
    )
set(POISSON_SOURCES
	"poisson8/factor.cpp" 
	"poisson8/geometry.cpp" 
	"poisson8/marching_cubes.cpp" 
	"poisson8/ply.cpp" 
	)
set(INCLUDE_FILES
			${SURFACE_INCLUDE_DIR}/helper.h
			${SURFACE_INCLUDE_DIR}/uniform_tsdf_volume.h 
			${SURFACE_INCLUDE_DIR}/scalable_tsdf_volume.h 
			${SURFACE_INCLUDE_DIR}/poisson_reconstruction.h 
			${POISSON_INCLUDES}
			)
set(SRC_FILES helper.cpp
			  uniform_tsdf_volume.cpp
			  scalable_tsdf_volume.cpp
			  poisson_reconstruction.cpp
			  ${POISSON_SOURCES}
			  )
source_group("Header Files\\poisson8" FILES ${POISSON_INCLUDES})
source_group("Source Files\\poisson8" FILES ${POISSON_SOURCES})

include_directories(${TBB_INCLUDE_DIRS})
add_library(${LIB_NAME} ${DVO_LIB_TYPE} ${INCLUDE_FILES} ${SRC_FILES})
set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries (${LIB_NAME} dvo_core ${OpenCV_LIBS} ${PCL_LIBRARIES} ${TBB_IMPORTED_TARGETS})