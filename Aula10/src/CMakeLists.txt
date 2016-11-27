
add_executable(FaceDetect FaceDetect.cpp)
target_link_libraries(FaceDetect ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(FaceRecog FaceRecog.cpp)
target_link_libraries(FaceRecog ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(TempMatch TempMatch.cpp)
target_link_libraries(TempMatch ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
