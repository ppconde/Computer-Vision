
add_executable(Hough Hough.cpp)
target_link_libraries(Hough ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(FeatDetect FeatDetect.cpp)
target_link_libraries(FeatDetect ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
