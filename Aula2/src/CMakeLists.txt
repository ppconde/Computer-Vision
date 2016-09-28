add_executable( DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )

add_executable( AddImages AddImages.cpp )
target_link_libraries( AddImages ${OpenCV_LIBS} )

add_executable( ChangeBrigtCront ChangeBrigtCront.cpp )
target_link_libraries( ChangeBrigtCront ${OpenCV_LIBS} )