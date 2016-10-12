#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int ratio = 3;		//ratio of lower to upper threshold
int const maxRatio = 5;
int ksize;			//kernel size
int lowThresh;		//Canny threshold
int const maxLowThresh = 200;

Mat frame, final, detEdges;

void CannyThresh(int, void*) {
	//applies blur filter
	blur(frame, detEdges, Size(3, 3));

	//uses Canny algorythm
	Canny(detEdges, detEdges, lowThresh, lowThresh*ratio, ksize);

	//uses Canny output as mask
	final = Scalar::all(0);

	//copies mask to frame
	frame.copyTo(final, detEdges);

	//displays result (shows pixels in "frame" that correspond to Canny output)
	imshow("Canny Edge Detection", final);
}

int main() {
	//opens default camera
	VideoCapture cap(0);
	
	//check if success
	if (!cap.isOpened()) return -1;

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl << "---------------------" << endl;
	cout << " Canny Edge Detector " << endl;
	cout << "---------------------" << endl << endl;
	cout << " Choose kernel size [3, 5 or 7]: ";
	cin >> ksize;

	namedWindow("Canny Edge Detection", 1);

	for(;;) {
		cap >> frame;								//gets a new frame from camera
		imshow("Video Capture", frame);				//shows original capture

		//creates matrix with same size and type as "frame"
		final.create(frame.size(), frame.type());

		//converts to grayscale
		cvtColor(frame, frame, CV_BGR2GRAY);

		//creates trackbar for user input
		createTrackbar("Min Threshold:", "Canny Edge Detection", &lowThresh, maxLowThresh, CannyThresh);
		createTrackbar("Thresh Ratio:", "Canny Edge Detection", &ratio, maxRatio, CannyThresh);
		
		//uses CannyThresh function, declared above
		CannyThresh(0, 0);

	    if (waitKey(5) >= 0) break;				//waits 5ms for program to render next frame
	}

	return 0;
}