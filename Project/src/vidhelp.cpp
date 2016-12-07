#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
Mat frame;
  //video file
	VideoCapture cap;
	const char* video = "vid/vid1.mp4";
  cap.open(video);

	//check if success
	if (!cap.isOpened()) return -1;


	for (;;) {
		cap >> frame;						//gets a new frame from camera

		if(!frame.data) break;

		imshow("teste", frame);

	    if (waitKey(500) >= 0) break;				//waits 5ms for program to render next frame
	}

	return 0;
}