#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define SMIN 30
#define VMIN 10
#define VMAX 256

Mat roiFrame, roiAux;
Rect roiBox;
vector<Point> roiPts;		//points defining a ROI
int cnt = 0;				//mouse clicks counter

static void roiSelection(int event, int x, int y, int, void*) {
	// This function waits for user to select points that define a ROI
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			cnt++;
			
			//point selection and ROI definition
			if (cnt <= 2) {
				//point selection and display
				Point selected = Point(x, y);
				roiPts.push_back(selected);

				circle(roiFrame, selected, 5, Scalar(0, 0, 255), 1);
				if (cnt == 2) {
					//ROI display and storage
					rectangle(roiFrame, roiPts[0], roiPts[1], Scalar(255, 0, 0), 2);
					roiBox = Rect(roiPts[0], roiPts[1]);
				}
			}
			else {
				//flushes point vector
				roiFrame = roiAux.clone();
				roiPts.clear();
				cnt = 0;
			}

			imshow("Object Tracking", roiFrame);			
	}
}

int main() {
	bool sel;

	Mat frame, hsv, hue, roi, mask, roiMask, roiHist, backProj;

	//video file
	const char* video = "vid/vid1.mp4";

	//histogram configurations
	const int channels = 0;
	int histSize = 16;
	float range[] = {0, 180};
	const float* histRange = {range};

	//presents user interface
	system("clear");		//clears terminal window
	cout
	<< endl
	<< " -----------------" << endl
	<< "  Object Tracking " << endl
	<< " -----------------" << endl
	<< endl
	<< " Choose to track object in video feed [0] or video file [1]: ";
	cin >> sel;
	cout << endl;

	//initializes video capture
	VideoCapture cap;
	if (sel) cap.open(video);
	else cap.open(0);

	if (!cap.isOpened()) return -1;

	//defines termination criteria for cam shift
	TermCriteria termCrit = TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1);

	//performs tracking
	for (;;) {
		cap >> frame;		//gets new frame

		//ends tracking if video ends
		if (frame.rows == 0 || frame.cols == 0) break;

		//converts frame to HSV colorspace
		cvtColor(frame, hsv, COLOR_BGR2HSV);

		//gets mask (easier and more accurate detection)
		inRange(hsv, Scalar(0, SMIN, MIN(VMIN, VMAX)),
           	Scalar(180, 256, MAX(VMIN, VMAX)), mask);

		int ch[] = {0, 0};
        hue.create(hsv.size(), hsv.depth());
        mixChannels(&hsv, 1, &hue, 1, ch, 1);

        //displays point selection (cnt == 0 by default)
		if (cnt == 0) {
			cout
			<< " Select two points to define ROI with the mouse. A third mouse click will reset the selection." << endl
			<< " Press ENTER when the selection is made." << endl << endl;

			//images for selection
			roiFrame = frame.clone();
			roiAux = roiFrame.clone();

			//mouse callback for selecting ROI
			imshow("Object Tracking", roiFrame);
			setMouseCallback("Object Tracking", roiSelection);
			waitKey(0);

			//is points have been selected and ROI defined
			if (roiPts.size() == 2) {
				//creates ROI and ROI mask
				roi = hue(roiBox);
				roiMask = mask(roiBox);

				//creates ROI histogram
				calcHist(&roi, 1, &channels, roiMask, roiHist, 1, &histSize, &histRange);
				normalize(roiHist, roiHist, 0, 255, NORM_MINMAX);
			}
			else {
				cout << " Error: not enough points selected to form ROI." << endl;
				return -1;
			}
		}
			
		//calculates histogram back projection
		calcBackProject(&hsv, 1, &channels, roiHist, backProj, &histRange);
		
		//backProj takes into account mask (better results)
		backProj &= mask;

		//calculates cam shift to track object inside ROI
		RotatedRect trackBox = CamShift(backProj, roiBox, termCrit);

		//draws ellipse around tracked object
		ellipse(frame, trackBox, Scalar(255, 0, 0), 2);
		
		imshow("Object Tracking", frame);

		if (waitKey(24) >= 0) break;
	}		
	return 0;
}