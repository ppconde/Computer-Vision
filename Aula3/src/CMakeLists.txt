add_executable( VideoCapture VideoCapture.cpp )
target_link_libraries( VideoCapture ${OpenCV_LIBS} )

add_executable( DetectSkin DetectSkin.cpp )
target_link_libraries( DetectSkin ${OpenCV_LIBS} )

add_executable( ImageFilter ImageFilter.cpp )
target_link_libraries( ImageFilter ${OpenCV_LIBS} )

add_executable( GetHist GetHist.cpp )
target_link_libraries( GetHist ${OpenCV_LIBS} )

add_executable( HistogramComp HistogramComp.cpp )
target_link_libraries( HistogramComp ${OpenCV_LIBS} )
