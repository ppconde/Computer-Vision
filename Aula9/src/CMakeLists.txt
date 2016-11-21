
#add_executable(OpticalFlow OpticalFlow.cpp)
#target_link_libraries(OpticalFlow ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(BackgForegSep BackgForegSep.cpp)
target_link_libraries(BackgForegSep ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(ObjTracking ObjTracking.cpp)
target_link_libraries(ObjTracking ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
add_executable(OpticalFlow OpticalFlow.cpp)
target_link_libraries(OpticalFlow ${OpenCV_LIBS} ${PCL_LIBRARIES})
#
