
add_executable(WriteToPCD WriteToPCD.cpp)
target_link_libraries(WriteToPCD ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(Reconstruct Reconstruct.cpp)
target_link_libraries(Reconstruct ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(ViewCloud ViewCloud.cpp)
target_link_libraries(ViewCloud ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(ViewOpenNI ViewOpenNI.cpp)
target_link_libraries(ViewOpenNI ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(ICPAlign ICPAlign.cpp)
target_link_libraries(ICPAlign ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
