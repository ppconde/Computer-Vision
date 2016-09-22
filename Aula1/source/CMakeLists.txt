add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )

add_executable( Color2Gray Color2Gray.cpp )
target_link_libraries( Color2Gray ${OpenCV_LIBS} )