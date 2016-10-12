#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	for(;;) {
		Mat frame;
		cap >> frame;								//gets a new frame from camera
		imshow("Video Capture", frame);				//shows original capture

	    cvtColor(frame, grayscale, CV_BGR2GRAY);	//converts to grayscale
	    
	    if (!type) imshow("Filtered Capture", grayscale);	//if grayscale selected, display
	    else {
	    	//outputs selected threshold
	    	if (method == 1) {
	    		threshold(grayscale, bw, 0, 255, THRESH_BINARY + THRESH_OTSU);
	    	}
	    	else if (method == 2) {
	    		adaptiveThreshold(grayscale, bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, threshValue, 0);
	    	}
	    	else if (method == 3) {
	    		adaptiveThreshold(grayscale, bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, threshValue, 0);
	    	}
	    	else threshold(grayscale, bw, threshValue, 255, THRESH_BINARY);

	    	imshow("Filtered Capture", bw);			//displays black & white image
		}

	    if (waitKey(5) >= 0) break;				//waits 30ms for program to render next frame
	}

	waitKey(0);
	
	return 0;
}