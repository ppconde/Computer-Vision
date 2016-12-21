#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Mat roiFrame, roiAux;
Rect roiBox;
vector<Point2f> roiPts;		//points defining a ROI
int cnt = 0;				//mouse clicks counter

int funcInt;	//for program function type


static void roiSelection(int event, int x, int y, int, void*) {
	// This function waits for user to select points that define a ROI
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			cnt++;

			//point selection and ROI definition
			if (cnt <= funcInt) {
				if (funcInt == 2) {
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
				else if (funcInt == 3) {
					//point selection and display
					Point first = Point(1, 1);
					roiPts.push_back(first);
					Point last = Point(roiFrame.size());
					roiPts.push_back(last);

					cnt = cnt + 2;

					Point selected = Point(x, y);
					roiPts.push_back(selected);

					circle(roiFrame, selected, 5, Scalar(0, 0, 255), 1);

					roiBox = Rect(roiPts[0], roiPts[1]);
				}
			}
			else {
				//flushes point vector
				roiFrame = roiAux.clone();
				roiPts.clear();
				cnt = 0;
			}

			imshow("Optical Flow", roiFrame);
	}
}

int main() {
	int sel;	//video selection
	bool func;	//program function type
	char file[20];
	int linesz;	//line thickness

	Mat frame, prevFrame, nextFrame, roi;

	vector<Point2f> prevPts, nextPts;	//frame features
	vector<uchar> status;				//status vector
	vector<float> err;					//error vector

	int nPts = 1000;					//number of features
	double qLevel = 0.05;				//quality level
	double minDist = 0.01;				//min euclidian distance

	float x1, y1, x2, y2;

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl
		 << " --------------------------" << endl
		 << "  Optical Flow for Project " << endl
		 << " --------------------------" << endl << endl
		 << " Choose video file [1 - ?]: ";
	cin >> sel;
	cout << endl;
	cout << " Choose [0 = ROI, 1 = single point]: ";
	cin >> func;
	cout << endl << endl;

	if (func) funcInt = 3;
	else funcInt = 2;

	//video file
	VideoCapture cap;

	sprintf(file, "vid/vid%d.mp4", sel);
	//const char* video = "vid/vid1.mp4";
	cap.open(file);

	//check if success
	if (!cap.isOpened()) return -1;

	for (;;) {
        //displays point selection (cnt == 0 by default)
		if (cnt == 0) {
			cout
			<< " Select two points to define ROI with the mouse. A third mouse click will reset the selection." << endl
			<< " Press ENTER when the selection is made." << endl << endl;

			cap >> frame;		//gets new frame
			//ends tracking if video ends
			if (frame.rows == 0 || frame.cols == 0) break;

			resize(frame, frame, Size(), 1.5, 1.5, INTER_CUBIC);
			cvtColor(frame, nextFrame, CV_BGR2GRAY);

			//images for selection
			roiFrame = frame.clone();
			roiAux = roiFrame.clone();

			//mouse callback for selecting ROI
			imshow("Optical Flow", roiFrame);
			setMouseCallback("Optical Flow", roiSelection);
			waitKey(0);

			//is points have been selected and ROI defined
			if (roiPts.size() == funcInt) {
				//creates ROI and ROI mask
				roi = frame(roiBox);
				cvtColor(roi, nextFrame, CV_BGR2GRAY);
				//cvtColor(roi, roi, COLOR_BGR2HSV);
				//imshow("roi", result);
				if (func) nextPts.push_back(roiPts[2]);
			}
			else {
				cout << " Error: not enough points selected to form ROI." << endl;
				return -1;
			}
		}

		//gets image features
		if (!func) {
			goodFeaturesToTrack(nextFrame, nextPts, nPts, qLevel, minDist);
		}

		prevFrame = nextFrame.clone();		//first frame is the same as last one
		prevPts = nextPts;
		cap >> frame;						//gets a new frame from camera

		resize(frame, frame, Size(), 1.5, 1.5, INTER_CUBIC);
		cvtColor(frame, nextFrame, CV_BGR2GRAY);

		nextFrame = nextFrame(roiBox);

		//calculates optical flow using Lucas-Kanade
		calcOpticalFlowPyrLK(prevFrame, nextFrame, prevPts, nextPts, status, err);

		//draw ROI rectangle
		if (!func) {
			rectangle(frame, roiPts[0], roiPts[1], Scalar(255, 255, 0), 1);
		}

		//draws motion lines on display
		for (int i = 0; i < nextPts.size(); i++) {
			if (status[i]) {
				x1 = roiPts[0].x + prevPts[i].x;
				y1 = roiPts[0].y + prevPts[i].y;
				x2 = roiPts[0].x + nextPts[i].x;
				y2 = roiPts[0].y + nextPts[i].y;

				cout << "PONTOS" << endl;
				cout << roiPts << endl;
				cout << prevPts[i] << endl;
				cout << nextPts[i] << endl << endl;

				cout << x1 << endl;
				cout << y1 << endl;
				cout << x2 << endl;
				cout << y2 << endl << endl;

				if (func) linesz = 2;
				else linesz = 1;

				line(frame, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 0), linesz);
			}
		}

		imshow("Optical Flow", frame);	//shows original capture

		//pauses and exits videos
		char key = waitKey(33);

		//if "space" pressed, pauses
	    if (key == 32) {
	    	cout << "test" << endl;
	    	key = 0;
	    	while (key != 32) {
	    		imshow("Optical Flow", frame);
	    		key = waitKey(5);
	    		if (key == 27) break;
	    	}
	    	if (key == 27) break;
	    	else key = 0;
	    }
	    //if "esc" pressed, exits
	    if (key == 27) break;
	}

	return 0;
}