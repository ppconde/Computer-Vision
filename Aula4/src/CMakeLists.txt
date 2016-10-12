
add_executable(GradCalc GradCalc.cpp)
target_link_libraries(GradCalc ${OpenCV_LIBS})
#
add_executable(Canny Canny.cpp)
target_link_libraries(Canny ${OpenCV_LIBS})
#
add_executable(CornerDetect CornerDetect.cpp)
target_link_libraries(CornerDetect ${OpenCV_LIBS})
#
add_executable(HoughLine HoughLine.cpp)
target_link_libraries(HoughLine ${OpenCV_LIBS})
#