#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int filter;
int size;

int main() {
	//opens default camera
	VideoCapture cap(0);
	
	//check if success
	if (!cap.isOpened()) return -1;

	//presents options to user
	system("clear");		//clears terminal window
	cout << endl << "--------------" << endl;
	cout << " Image Filter " << endl;
	cout << "--------------" << endl << endl;
	cout << " Choose image filter:" << endl;
	cout << "  blur = 0\n  median blur = 1\n  gaussian blur = 2" << endl;
	cin >> filter;

	cout << endl << " Enter filter size [1-255] (only odd numbers): ";
	cin >> size;
	
	//creates image objects and windows
	Mat result;

	namedWindow("Video Capture", 1);
	namedWindow("Filtered Capture", 1);

	//displays original video capture and alternatives
	for(;;) {
		Mat frame;
		cap >> frame;								//gets a new frame from camera
		imshow("Video Capture", frame);				//shows original capture

	    //cvtColor(frame, grayscale, CV_BGR2GRAY);	//converts to grayscale

	   	//applies selected filter and displays result
	   	switch (filter) {
	   		case 0:
	   			blur(frame, result, Size(size, size), Point(-1, -1), BORDER_DEFAULT);
	   			break;
	   		case 1:
	   			medianBlur(frame, result, size);
	   			break;
	   		case 2:
	   			GaussianBlur(frame, result, Size(size, size), 0, 0, BORDER_DEFAULT);
	   			break;
	   		default:
	   			cout << " No valid image filter selected." << endl;
	   			return -1;
	   	}

    	imshow("Filtered Capture", result);			//displays black & white image

	    if (waitKey(5) >= 0) break;			//waits 30ms for program to render next frame
	}
	
	return 0;
}