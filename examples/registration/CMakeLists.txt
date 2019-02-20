set(LINK_LIBS dvo_core dvo_registration ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(example_tracking example_tracking.cpp)
target_link_libraries(example_tracking ${LINK_LIBS})
