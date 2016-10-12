#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int thresh = 100;		//threshold level
int const maxThresh = 255;

Mat frame, final;

void cornerDetect(int, void*) {
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;

	Mat finalScaled;

	//Harris algorythm for corner detection
	cornerHarris(frame, final, blockSize, apertureSize, k, BORDER_DEFAULT);

	normalize(final, final, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(final, finalScaled);

	//draws circles around corners
	for(int j = 0; j < final.rows; j++) {
		for(int i = 0; i < final.cols; i++) {
    		if((int) final.at<float>(j, i) > thresh) {
            	circle(finalScaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
            }
        }
    }

	//displays result (shows pixels in "frame" that correspond to Canny output)
	imshow("Corner Detection", finalScaled);
}

int main() {
	//opens default camera
	VideoCapture cap(0);
	
	//check if success
	if (!cap.isOpened()) return -1;

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl << "------------------" << endl;
	cout << " Corner Detection " << endl;
	cout << "------------------" << endl << endl;

	namedWindow("Corner Detection", 1);

	cap >> frame;								//gets a new frame from camera
	imshow("Video Capture", frame);				//shows original capture

	//converts to grayscale
	cvtColor(frame, frame, CV_BGR2GRAY);

	//creates matrix with same size and type as "frame"
	final.create(frame.size(), frame.type());

	//creates trackbar for user input
	createTrackbar("Threshold:", "Corner Detection", &thresh, maxThresh, cornerDetect);
		
	//uses CannyThresh function, declared above
	cornerDetect(0, 0);

	waitKey(0);

	return 0;
}