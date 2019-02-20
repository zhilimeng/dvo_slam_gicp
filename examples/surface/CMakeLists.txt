set(LINK_LIBS dvo_core dvo_surface ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(example_poisson_fusion example_poisson_fusion.cpp)
target_link_libraries(example_poisson_fusion ${LINK_LIBS})

add_executable(example_tsdf_fusion example_tsdf_fusion.cpp)
target_link_libraries(example_tsdf_fusion ${LINK_LIBS})