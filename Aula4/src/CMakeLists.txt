add_executable( HoughCircle HoughCircle.cpp )
target_link_libraries( HoughCircle ${OpenCV_LIBS} )
add_executable( HoughLine HoughLine.cpp )
target_link_libraries( HoughLine ${OpenCV_LIBS} )
