
add_executable( chessboard chessboard.cpp )
target_link_libraries( chessboard ${OpenCV_LIBS} )
#
add_executable( StereoCalib StereoCalib.cpp )
target_link_libraries( StereoCalib ${OpenCV_LIBS} )
#
add_executable( Undistort Undistort.cpp )
target_link_libraries( Undistort ${OpenCV_LIBS} )
#
add_executable( Rectify Rectify.cpp )
target_link_libraries( Rectify ${OpenCV_LIBS} )
#
