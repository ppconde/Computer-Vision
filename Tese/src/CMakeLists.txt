
add_executable(Thesis Thesis.cpp)
target_link_libraries(Thesis ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

add_executable(Mouse Mouse.cpp)
target_link_libraries(Mouse ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
