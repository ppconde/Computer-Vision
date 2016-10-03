#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

bool type;
int method;
int threshValue;

int main() {
	//opens default camera
	VideoCapture cap(0);
	
	//check if success
	if (!cap.isOpened()) return -1;

	//presents options to user
	system("clear");		//clears terminal window
	cout << endl << "---------------" << endl;
	cout << " Video Capture " << endl;
	cout << "---------------" << endl << endl;
	cout << " Choose image output type (grayscale = 0, black & white = 1: ";
	cin >> type;

	if (type) {
		cout << " Choose threshold method (global = 0, median = 1, gaussian = 2): ";
		cin >> method;	
	}
	
	//creates image objects and windows
	Mat image;
	Mat grayscale;
	Mat bw;

	namedWindow("Video Capture", 1);
	namedWindow("Filtered Capture", 1);

	//asks for user input regarding filter size
	if (type) {
		if (!method) cout << " Enter threshold value [0-255]: ";
		else cout << " Enter block size [3-255] (only odd numbers): ";
		cin >> threshValue;
	}

	//displays original video capture and alternatives
	for(;;) {
		Mat frame;
		cap >> frame;								//gets a new frame from camera
		imshow("Video Capture", frame);				//shows original capture

	    cvtColor(frame, grayscale, CV_BGR2GRAY);	//converts to grayscale
	    
	    if (!type) imshow("Filtered Capture", grayscale);	//if grayscale selected, display
	    else {
	    	//outputs selected threshold
	    	if (method == 1) {
	    		adaptiveThreshold(grayscale, bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, threshValue, 0);
	    	}
	    	else if (method == 2) {
	    		adaptiveThreshold(grayscale, bw, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, threshValue, 0);
	    	}
	    	else threshold(grayscale, bw, threshValue, 255, THRESH_BINARY);

	    	imshow("Filtered Capture", bw);			//displays black & white image
		}

	    if (waitKey(5) >= 0) break;				//waits 30ms for program to render next frame
	}
	
	return 0;
}