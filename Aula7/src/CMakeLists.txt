
add_executable(WriteToPCD WriteToPCD.cpp)
target_link_libraries(WriteToPCD ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(reconstruct reconstruct.cpp)
target_link_libraries(reconstruct ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(viewCloud viewCloud.cpp)
target_link_libraries(viewCloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(viewOpenni viewOpenni.cpp)
target_link_libraries(viewOpenni ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
