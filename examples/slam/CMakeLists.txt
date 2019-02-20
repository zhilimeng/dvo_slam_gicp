set(LINK_LIBS dvo_core dvo_registration dvo_slam ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(example_tum_rgbd example_tum_rgbd.cpp)
target_link_libraries(example_tum_rgbd ${LINK_LIBS})