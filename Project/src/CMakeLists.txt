
add_executable(Project Project.cpp)
target_link_libraries(Project ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(Histogram Histogram.cpp)
target_link_libraries(Histogram ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(Kmeans Kmeans.cpp)
target_link_libraries(Kmeans ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
