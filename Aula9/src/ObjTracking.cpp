#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define VEC_SIZE 4

Mat roiFrame;
vector<Point> roiPts;		//points defining a ROI

void roiSelection(int event, int x, int y, int flags, void* param) {
	// This function waits for user to select points that define a ROI
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			//point selection and storage
			if (roiPts.size() < VEC_SIZE) {
				Point selected = Point(x, y);
				roiPts.push_back(selected);

				circle(roiFrame, selected, 5, Scalar(0, 0, 255), 1);
			}

			imshow("Select ROI", roiFrame);
	}
}

int main() {
	bool sel;

	Mat frame, prevFrame, nextFrame, roi, roiHist, backProj;

	const char* video = "vid/vid1.mp4";

	//histogram configurations
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

	//gets first frame
	cap >> frame;
	roiFrame = frame.clone();

	//mouse callback for selecting ROI
	imshow("Select ROI", roiFrame);
	setMouseCallback("Select ROI", roiSelection);
	waitKey(0);
	//destroyWindow("Select ROI");

	int roiBox = roiPts.size();

	if (roiBox == 4) {
		//creates ROI - MUDAR PARA PROCURAR MAIOR E MENOR PONTO
		Vec2i tl = roiPts[0];		//top left corner
		Vec2i br = roiPts[3];		//bottom right corner
		roi = frame(Rect(roiPts[0], roiPts[3]));

		cvtColor(roi, roi, COLOR_BGR2HSV);

		//creates ROI histogram
		calcHist(&roi, 1, 0, Mat(), roiHist, 1, &histSize, &histRange);
		normalize(roiHist, roiHist, 0, 255, NORM_MINMAX, -1, Mat());

		//defines termination criteria for cam shift
		//TermCriteria termCrit = TermCriteria(TERM_CRITERIA_EPS | TERM_CRITERIA_COUNT, 10, 1);

		//performs tracking
		for (;;) {
			prevFrame = nextFrame.clone();	//copies last frame
			cap >> frame;					//gets new frame

			//ends tracking if video ends
			if (frame.rows == 0 || frame.cols == 0) break;

			cvtColor(frame, frame, COLOR_BGR2HSV);

			//calculates histogram back projection
			float range[] = {0, 180};
			const float* frameRange = {range};
			calcBackProject(&frame, 1, 0, roiHist, backProj, &frameRange);

			//calculates cam shift
			RotatedRect trackBox = CamShift(backProj, roi,
				TermCriteria(TERM_CRITERIA_EPS | TERM_CRITERIA_COUNT, 10, 1));

			imshow("Object Tracking", frame);

			if (waitKey(24) >= 0) break;
		}
	}
	else {
		cout << " Error: not enough points selected to form ROI." << endl;
		return -1;
	}

	return 0;
}