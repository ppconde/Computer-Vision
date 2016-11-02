
add_executable(WriteToPCD WriteToPCD.cpp)
target_link_libraries(WriteToPCD ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
#
