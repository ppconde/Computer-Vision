#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
	Mat frame, prevFrame, nextFrame;		//image matrixes
	
	vector<Point2f> prevPts, nextPts;		//frame features
	vector<uchar> status;					//status vector
	vector<float> err;						//error vector

	int nPts = 500;						//number of features
	double qLevel = 0.05;				//quality level
	double minDist = 1.0;				//min euclidian distance

	//opens default camera
	VideoCapture cap("vid/Bike.avi");

	//check if success
	if (!cap.isOpened()) return -1;

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl << " --------------------------------" << endl;
	cout << "  Optical Flow with Lucas-Kanade " << endl;
	cout << " --------------------------------" << endl << endl;

	namedWindow("Optical Flow L-K", 1);

	//gets initial image
	cap >> frame;
	cvtColor(frame, nextFrame, CV_BGR2GRAY);

	for (;;) {
		//gets image features
		goodFeaturesToTrack(nextFrame, nextPts, nPts, qLevel, minDist);
		
		prevFrame = nextFrame.clone();		//first frame is the same as last one
		prevPts = nextPts;
		cap >> frame;						//gets a new frame from camera
		
		if (!frame.data) {
			cap.open("vid/Bike.avi");
			continue;
		}

		cvtColor(frame, nextFrame, CV_BGR2GRAY);

		//calculates optical flow using Lucas-Kanade
		calcOpticalFlowPyrLK(prevFrame, nextFrame, prevPts, nextPts, status, err);

		//draws motion lines on display
		for (int i = 0; i < nextPts.size(); i++) {
			if (status[i]) {
				line(frame, prevPts[i], nextPts[i], Scalar(255, 0, 0));
			}
		}

		imshow("Optical Flow L-K", frame);	//shows original capture

	    if (waitKey(5) >= 0) break;				//waits 5ms for program to render next frame
	}

	return 0;
}