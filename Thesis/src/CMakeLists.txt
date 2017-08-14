PYTHON_ADD_MODULE(mor
add_executable(Thesis Thesis.cpp)
target_link_libraries(Thesis ${PYTHON_LIBRARIES} ${PYTHON2_INCLUDE_DIRS} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(interpreter interpreter.cpp)
target_link_libraries(interpreter ${PYTHON_LIBRARIES} ${PYTHON2_INCLUDE_DIRS} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
#add_executable(blob blob.cpp)
#target_link_libraries(blob ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(snake snake.cpp)
target_link_libraries(snake ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

#add_executable(Mouse Mouse.cpp)
#target_link_libraries(Mouse ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

#add_executable(Poly Poly.cpp)
#target_link_libraries(Poly ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

#add_executable(Ellipse Ellipse.cpp)
#target_link_libraries(Ellipse ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

#add_executable(Contour Contour.cpp)
#target_link_libraries(Contour ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

#add_executable(Hull Hull.cpp)
#target_link_libraries(Hull ${OpenCV_LIBS} ${PCL_LIBRARIES})
#

#add_executable(1 1.cpp)
#target_link_libraries(1 ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
