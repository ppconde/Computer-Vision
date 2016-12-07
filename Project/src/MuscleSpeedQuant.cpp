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

static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}

/*static void roiSelection(int event, int x, int y, int, void*) {
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
}*/

int main() {
	bool sel;

	//Mat frame, hsv, roi, mask, roiMask, roiHist, backProj;

	//video file
	const char* video = "vid/vid1.mp4";

	//histogram configurations
	int channels[] = {0, 1};
	int hbins = 30;
	int sbins = 32;
	int histSize[] = {hbins, sbins};
	float hrange[] = {0, 180};
	float srange[] = {0, 256};
	const float* histRange[] = {hrange, srange};

	//presents user interface
	system("clear");		//clears terminal window
	cout
	<< endl
	<< " ---------" << endl
	<< "  Project " << endl
	<< " ---------" << endl
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
	
	Mat flow, cflow, frame;
	UMat gray, prevGray, uflow;
	namedWindow("flow", 1);
	//performs tracking
	for (;;) {
		cap >> frame;		//gets new frame

		//ends tracking if video ends
		//if (frame.rows == 0 || frame.cols == 0) break;

		//converts frame to HSV colorspace
		/*cvtColor(frame, hsv, COLOR_BGR2HSV);

		//gets mask (easier and more accurate detection)
		inRange(hsv, Scalar(0, SMIN, MIN(VMIN, VMAX)),
           	Scalar(180, 256, MAX(VMIN, VMAX)), mask);

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
				roi = hsv(roiBox);
				roiMask = mask(roiBox);
				imshow("roi", roi);
				imshow("roiMask", roiMask);
				
				//creates ROI histogram
				//(using both hue and sat (2 channels) gives better results)
				calcHist(&roi, 1, channels, roiMask, roiHist, 2, histSize, histRange);
				normalize(roiHist, roiHist, 0, 255, NORM_MINMAX);
			}
			else {
				cout << " Error: not enough points selected to form ROI." << endl;
				return -1;
			}
		}

		//calculates histogram back projection
		calcBackProject(&hsv, 1, channels, roiHist, backProj, histRange);

		//backProj takes into account mask (better results)
		backProj &= mask;

		//calculates cam shift to track object inside ROI
		RotatedRect trackBox = CamShift(backProj, roiBox, termCrit);

		//draws ellipse around tracked object
		ellipse(frame, trackBox, Scalar(255, 0, 0), 2);*/
		
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		if (!prevGray.empty()) {
			calcOpticalFlowFarneback(prevGray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
			cvtColor(prevGray, cflow, COLOR_BGR2GRAY);
			uflow.copyTo(flow);
			drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
			imshow("flow", cflow);
		}
		if (waitKey(30) >= 0) break;

		swap(prevGray, gray);

		//imshow("Object Tracking", frame);

		//if (waitKey(24) >= 0) break;
	}
	return 0;
}
