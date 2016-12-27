#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define STEP 		10
#define FLOWSZ 		0.5
#define FB_SCALE	10

Mat roiFrame, roiAux;
Rect roiBox;
vector<Point2f> roiPts;		//points defining a ROI
unsigned int cnt = 0;				//mouse clicks counter

unsigned int funcInt;	//for program function type


static void roiSelection(int event, int x, int y, int, void*) {
	// This function waits for user to select points that define a ROI or a point of interest
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
	int sel;							//video selection
	int func;							//program function type
	char file[20];
	int linesz;							//line thickness

	Mat frame, prevFrame, nextFrame, roi, flow;
	UMat flowUMat;

	vector<Point2f> prevPts, nextPts;	//frame features
	vector<uchar> status;				//status vector
	vector<float> err;					//error vector

	Point2f flowPt;

	int nPts = 2000;					//number of features
	double qLevel = 0.05;				//quality level
	double minDist = 0.01;				//min euclidian distance

	float x1, y1, x2, y2;				//point coordinates for drawing
	vector<Point2f> linPts;				//auxilliary vector for point path

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl
		 << " --------------------------------" << endl
		 << "  Muscle Movement Quantification " << endl
		 << " --------------------------------" << endl << endl
		 << " Choose video file [1 - ?]: ";
	cin >> sel;
	cout << endl;
	cout << " Choose [0 = ROI (L-K), 1 = single point (L-K), 2 = Grid (FB)]: ";
	cin >> func;
	cout << endl;

	//properties for each program function
	if (func == 0) {
		funcInt = 2;
		linesz = 1;

		cout
		<< " Select two points to define ROI with the mouse. A third mouse click will reset the selection." << endl
		<< " Press ENTER when the selection is made." << endl << endl;

	}
	else if (func == 1) {
		funcInt = 3;
		linesz = 2;

		cout
		<< " Select one point of interest with the mouse. A second mouse click will reset the selection." << endl
		<< " Press ENTER when the selection is made." << endl << endl;
	}
	else if (func == 2) {
		cout << " Farneback selected" << endl << endl;
	}

	//video file
	VideoCapture cap;

	sprintf(file, "vid/vid%d.mp4", sel);
	//const char* video = "vid/vid1.mp4";
	cap.open(file);

	//check if success
	if (!cap.isOpened()) {
		cout << " Error: video file could not be loaded. Aborting." << endl << endl;
		return -1;
	}

	for (;;) {
		//initial behavior
		if (cnt == 0) {
			cap >> frame;		//gets new frame
			if(!frame.data) {
				cout << " Error: no video data found." << endl;
				break;
			}

			//resizes video frames for viewing purposes
			resize(frame, frame, Size(), 1.5, 1.5, INTER_CUBIC);

			//converts frame to grayscale
			cvtColor(frame, nextFrame, CV_BGR2GRAY);

			if (func == 0 || func == 1) {
				//images for selection
				roiFrame = frame.clone();
				roiAux = roiFrame.clone();

				//mouse callback for selecting ROI
				imshow("Optical Flow", roiFrame);
				setMouseCallback("Optical Flow", roiSelection);
				waitKey(0);

				//if points have been selected and ROI defined
				if (roiPts.size() == funcInt) {
					//creates ROI and ROI mask
					roi = frame(roiBox);
					cvtColor(roi, nextFrame, CV_BGR2GRAY);

					if (func) {
						nextPts.push_back(roiPts[2]);
						linPts.push_back(roiPts[2]);
					}
				}
				else {
					cout << " Error: not enough points selected to form ROI." << endl;
					return -1;
				}
			}
			else {
				prevFrame = nextFrame.clone();

				//overrides counter
				cnt = 1;
			}
		}

		if (func == 0) {
			//gets image features
			goodFeaturesToTrack(nextFrame, nextPts, nPts, qLevel, minDist);
		}

		if (func == 0 || func == 1) prevPts = nextPts;

		prevFrame = nextFrame.clone();		//first frame is the same as last one
		cap >> frame;						//gets a new frame from camera
			
		//ends tracking if video ends
		if(!frame.data) {
			cout << " Video has ended. Tracking Stopped." << endl << endl;
			break;
		}

		resize(frame, frame, Size(), 1.5, 1.5, INTER_CUBIC);
		cvtColor(frame, nextFrame, CV_BGR2GRAY);

		if (func == 0 || func == 1) {
			nextFrame = nextFrame(roiBox);

			//calculates optical flow using Lucas-Kanade
			calcOpticalFlowPyrLK(prevFrame, nextFrame, prevPts, nextPts, status, err);

			if (func == 0) {
				//draws ROI rectangle
				rectangle(frame, roiPts[0], roiPts[1], Scalar(255, 255, 0), 1);
				
				//draws motion lines on display
				for (unsigned int i = 0; i < nextPts.size(); i++) {
					if (status[i]) {
						x1 = roiPts[0].x + prevPts[i].x;
						y1 = roiPts[0].y + prevPts[i].y;
						x2 = roiPts[0].x + nextPts[i].x;
						y2 = roiPts[0].y + nextPts[i].y;

						/*cout << "PONTOS" << endl;
						cout << roiPts << endl;
						cout << prevPts[i] << endl;
						cout << nextPts[i] << endl << endl;

						cout << x1 << endl;
						cout << y1 << endl;
						cout << x2 << endl;
						cout << y2 << endl << endl;*/

						line(frame, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 0), linesz);
					}
				}
			}
			else {
				//stores points in a new vector
				linPts.push_back(nextPts[0]);
				//cout << linPts << endl;
				//cout << linPts.size() << endl << endl;

				//draws the path of selected point
				for (unsigned int i = 0; i < linPts.size() - 1; i++) {
					if (status[0]) {
						//line(frame, linPts[i], linPts[i+1], Scalar(255, 255, 0), linesz);
							
						if (i == linPts.size() - 2) {
							//draws at current point position
							line(frame, linPts[i], linPts[i+1], Scalar(0, 0, 255), linesz);
							//circle(frame, linPts[i+1], 5, Scalar(0, 0, 255), 1);
						}
						else {
							line(frame, linPts[i], linPts[i+1], Scalar(255, 255, 0), linesz);
						}
					}
				}
			}
		}
		else {
			//calculates optical flow using Farneback
			calcOpticalFlowFarneback(prevFrame, nextFrame, flowUMat, 0.5, 3, 15, 3, 5, 1.2, 0);

			flowUMat.copyTo(flow);

			//creates flow analysis grid
			for (int i = 0; i < frame.rows; i += STEP) {
				for (int j = 0; j < frame.cols; j += STEP) {
					flowPt = flow.at<Point2f>(i, j)*FB_SCALE;

					if ((flowPt.x > FLOWSZ || flowPt.x < -FLOWSZ) && (flowPt.y > FLOWSZ || flowPt.y < -FLOWSZ)) {
						line(frame, Point(j, i), Point(j + flowPt.x, i + flowPt.y), Scalar(255, 255, 0));
					}
					else continue;
				}
			}
		}

		imshow("Optical Flow", frame);	//shows original capture

		//pauses and exits videos
		char key = waitKey(33);

		//if "space" pressed, pauses. If "esc" pressed, exits program
	    if (key == 32) {
	    	key = 0;

	    	while (key != 32) {
	    		imshow("Optical Flow", frame);
	    		key = waitKey(5);
	    		if (key == 27) break;
	    	}
	    	if (key == 27) {
	    		cout << " Tracking aborted by user. Exiting." << endl << endl;
	    		break;
	    	}
	    	else key = 0;
	    }
	    //if "esc" pressed, exits
	    if (key == 27) break;
	}

	return 0;
}