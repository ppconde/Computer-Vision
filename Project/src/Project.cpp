#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define RES_SCALE	1.5
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
					//point selection
					Point selected = Point(x, y);
					roiPts.push_back(selected);

					//displays selected point
					circle(roiFrame, selected, 5, Scalar(0, 0, 255), 1);
					
					if (cnt == 2) {
						//ROI display and storage
						rectangle(roiFrame, roiPts[0], roiPts[1], Scalar(255, 0, 0), 2);
						roiBox = Rect(roiPts[0], roiPts[1]);
					}
				}
				else if (funcInt == 3) {
					//defines ROI as whole frame
					Point first = Point(1, 1);
					roiPts.push_back(first);
					Point last = Point(roiFrame.size());
					roiPts.push_back(last);

					cnt = cnt + 2;

					//point selection
					Point selected = Point(x, y);
					roiPts.push_back(selected);

					//displays selected point
					circle(roiFrame, selected, 5, Scalar(0, 0, 255), 1);

					//stores ROI
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

int ColorSeg(Point2f pt) {
	// This function attributes a color to a vector, taking into account its orientation
	int a;
	//South-East = YELLOW
	if (pt.x > 0 && pt.y > 0) a = 0;
	//South-West = CYAN
	else if (pt.x < 0 && pt.y > 0) a = 1;
	//North-West = MAGENTA
	else if (pt.x < 0 && pt.y < 0) a = 2;
	//North-East = GREEN
	else if (pt.x > 0 && pt.y < 0) a = 3;
	//other orientations or no orientation = WHITE
	else a = 4;

	return a;
}

int main(int argc, char** argv) {
	int sel;							//video selection
	int func;							//program function type
	bool traj = 0;						//trajectory visualization
	char file[20];
	int linesz;							//line thickness

	//color structure for segmentation
	Scalar color[] =
    {
        Scalar(0, 255, 255),			//yellow
        Scalar(255, 255, 0),			//cyan
        Scalar(255, 0, 255), 			//magenta
        Scalar(0, 255, 0),				//green
        Scalar(255, 255, 255)			//white
    };
    int cidx;							//color index

	//matrixes declaration
	Mat frame, prevFrame, nextFrame, roi, flow;
	UMat flowUMat;

	//vectors  and points declaration
	vector<Point2f> oriVec;				//orientations vector
	vector<Point2f> prevPts, nextPts;	//Point to analyze
	vector<uchar> status;				//L-K status vector
	vector<float> err;					//L-K error vector
	Point2f flowPt;						//Farneback flow point

	//L-K optical flow configuration
	int nPts = 5000;					//number of features
	double qLevel = 0.01;				//quality level
	double minDist = 0.01;				//min euclidian distance

	//Farneback optical flow configuration
	float pyrSc = 0.5;					//pyramid scale
	int levels = 2;						//pyramid levels
	int winsz = 15;						//averaging window size
	int iters = 2;						//number of iterations
	int polyN = 5;						//pix neighborhood size for polynomial expansion
	float polyS = 1.2;					//gaussian std dev for polynomial expansion

	float x1, y1, x2, y2;				//point coords for drawing

	//presents user interface
	system("clear");		//clears terminal window
	cout << endl
		//title
		 << " --------------------------------" << endl
		 << "  Muscle Movement Quantification " << endl
		 << " --------------------------------" << endl
		 << endl
		//help
		 << " ----- Help -----" << endl
		 << " After the needed configuration, which will be prompted to the user, a" << endl
		 << "window with the video feed will be displayed." << endl
		 << " The following commands are available:" << endl
		 << "        Space\t- pauses the video" << endl
		 << "        S\t- shows movement trajectories (hidden by default)" << endl
		 << "        Esc\t- exits the program" << endl
		 << " The movement quantification is color coded:" << endl
		 << "        Green\t- NE (North-East)" << endl
		 << "        Magenta\t- NW (North-West)" << endl
		 << "        Cyan\t- SW (South-West)" << endl
		 << "        Yellow\t- SE (South-East)" << endl
		 << " ----------------" << endl
		 << endl;

	//video file selection
	if (argc == 2) sprintf(file, "%s", argv[1]);
	else {
		cout << " No input argument given." << endl
		 	 << " Choose example video file [1, 2]: ";
		cin >> sel;
		cout << endl;

		sprintf(file, "vid/vid%d.mp4", sel);
	}

	//video capture declaration
	VideoCapture cap;

	cap.open(file);

	//check if success
	if (!cap.isOpened()) {
		cout << " Error: video file could not be loaded. Aborting." << endl << endl;
		return -1;
	}

	//analysis method selection
	cout << " Choose analysis method [0 = ROI (L-K), 1 = point (L-K), 2 = grid (FB)]: ";
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
		linesz = 1;

		cout
		<< " Select one point of interest with the mouse. A second mouse click will reset the selection." << endl
		<< " Press ENTER when the selection is made." << endl << endl;
	}
	else if (func == 2) {
		linesz = 1;
		cout << " Farneback selected" << endl << endl;
	}
	else {
		cout << " Error: invalid method choosen. Aborting." << endl << endl;
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
			resize(frame, frame, Size(), RES_SCALE, RES_SCALE, INTER_CUBIC);

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
						oriVec.push_back(roiPts[2]);
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

		resize(frame, frame, Size(), RES_SCALE, RES_SCALE, INTER_CUBIC);
		cvtColor(frame, nextFrame, CV_BGR2GRAY);

		if (func == 0 || func == 1) {
			nextFrame = nextFrame(roiBox);

			//calculates optical flow using Lucas-Kanade
			calcOpticalFlowPyrLK(prevFrame, nextFrame, prevPts, nextPts, status, err);

			if (func == 0) {
				//calculates orientations vector
				subtract(nextPts, prevPts, oriVec);

				//draws ROI rectangle
				rectangle(frame, roiPts[0], roiPts[1], Scalar(255, 255, 0), 1);

				//draws motion lines on display
				if (traj) {
					for (unsigned int i = 0; i < nextPts.size(); i++) {
						if (status[i]) {
							/*x1 = roiPts[0].x + prevPts[i].x;
							y1 = roiPts[0].y + prevPts[i].y;
							x2 = roiPts[0].x + nextPts[i].x;
							y2 = roiPts[0].y + nextPts[i].y;*/

							x1 = roiPts[0].x + prevPts[i].x;
							y1 = roiPts[0].y + prevPts[i].y;
							x2 = x1 + oriVec[i].x*5;
							y2 = y1 + oriVec[i].y*5;

							cidx = ColorSeg(oriVec[i]);
		
							line(frame, Point(x1, y1), Point(x2, y2), color[cidx], linesz);
						}
					}
				}
			}
			else {
				//stores points in a new vector
				oriVec.push_back(nextPts[0]);

				//draws the path of selected point
				for (unsigned int i = 0; i < oriVec.size() - 1; i++) {
					if (status[0]) {	
						if (i == oriVec.size() - 2) {
							//draws at current point position
							line(frame, oriVec[i], oriVec[i+1], Scalar(0, 0, 255), linesz);
							//circle(frame, oriVec[i+1], 5, Scalar(0, 0, 255), 1);
						}
						else {
							line(frame, oriVec[i], oriVec[i+1], Scalar(255, 255, 0), linesz);
						}
					}
				}
			}
		}
		else {
			//calculates optical flow using Farneback
			calcOpticalFlowFarneback(prevFrame, nextFrame, flowUMat, pyrSc, levels, winsz, iters, polyN, polyS, 0);

			flowUMat.copyTo(flow);

			//flushes orientations vector for new frame
			oriVec.clear();

			//creates flow analysis grid
			for (int i = 0; i < frame.rows; i += STEP) {
				for (int j = 0; j < frame.cols; j += STEP) {
					flowPt = flow.at<Point2f>(i, j)*FB_SCALE;

					//stores flow in orientations vector
					oriVec.push_back(flowPt);

					if (traj) {
						if ((flowPt.x > FLOWSZ || flowPt.x < -FLOWSZ) && (flowPt.y > FLOWSZ || flowPt.y < -FLOWSZ)) {
							cidx = ColorSeg(flowPt);

							line(frame, Point(j, i), Point(j + flowPt.x, i + flowPt.y), color[cidx], linesz);
						}
						else continue;
					}
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
	    //if "S" pressed, shows trajectories
	    if (key == 's' || key == 'S') traj = !traj;
	    //if "esc" pressed, exits
	    if (key == 27) break;
	}

	return 0;
}