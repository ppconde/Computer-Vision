#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
	bool colorSpace;

	//opens default camera
	VideoCapture cap(0);
	
	//check if success
	if (!cap.isOpened()) return -1;

	//presents options to user
	system("clear");		//clears terminal window
	cout << endl << "----------------" << endl;
	cout << " Skin Detection " << endl;
	cout << "----------------" << endl << endl;
	cout << " Choose color space (HSV = 0, YCrCb = 1): ";
	cin >> colorSpace;

	//creates image objects and windows
	Mat skin;

	namedWindow("Video Capture", 1);
	namedWindow("Skin Detection", 1);
	
	//skin detection
	for(;;) {
		Mat frame;
		cap >> frame;
		imshow("Video Capture", frame);

		if (colorSpace) {
			cvtColor(frame, skin, CV_BGR2YCrCb);							//converts to YCrCb
			inRange(skin, Scalar(0, 133, 77), Scalar(255, 173, 127), skin);	//finds values range
		}
		else {
			cvtColor(frame, skin, CV_BGR2HSV);								//converts to HSV
			inRange(skin, Scalar(0, 10, 60), Scalar(20, 150, 255), skin);
		}

		cvtColor(skin, skin, CV_GRAY2BGR);
		addWeighted(frame, 1, skin, 1, 0.0, skin);
		imshow("Skin Detection", skin);

		if (waitKey(5) >= 0) break;
	}
	return 0;
}