/*===========================================*/
/*											 */	
/*		MUSCLE MOVEMENT QUANTIFICATION 		 */
/*			COMPUTER VISION PROJECT 		 */
/*											 */
/*	Authors: Andre Saraiva					 */
/*											 */
/*	Date:	 January 2017					 */
/*											 */
/*	Libraries used: OpenCV 3.1.0			 */
/*											 */
/*===========================================*/


#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>

//namespace declaration
using namespace std;
using namespace cv;

//pre-processor defines
#define RES_SCALE	1.5		//image resizing scale
#define STEP 		10		//Farneback grid spacing in pix
#define FLOW_SZ 	0.5		//minimum flow size
#define LK_SCALE	5		//Lucas-Kanade flow scale
#define FB_SCALE	10		//Farneback flow scale

#define c2str(x)	#x

//global variables
Mat roiFrame, roiAux;
Rect roiBox;
vector<Point2f> roiPts;		//points defining a ROI
unsigned int cnt = 0;		//mouse clicks counter
unsigned int funcInt;		//for program function type

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


//functions' declaration
static void roiSelection(int, int, int, int, void*);

int PointLoc(Point2f);

void drawCompass(Mat&);

void getHist(Mat&, vector<Point2f>);


//main routine
int main(int argc, char** argv) {
	int sel;							//video selection
	int func;							//program function type
	bool traj = 0;						//trajectory visualization
	char file[20];
	int linesz;							//line thickness

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
	int flowDen;						//farneback flow density
	float pyrSc = 0.5;					//pyramid scale
	int levels = 2;						//pyramid levels
	int winsz = 15;						//averaging window size
	int iters = 2;						//number of iterations
	int polyN = 5;						//pix neighborhood size for polynomial expansion
	float polyS = 1.2;					//gaussian std dev for polynomial expansion
	int FBstep;							//grid spacing

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
		cout << endl
			 << " No input argument given." << endl
		 	 << " Choose example video file [1, 2]: ";
		cin >> sel;

		sprintf(file, "vid/vid%d.mp4", sel);
	}

	//video capture declaration
	VideoCapture cap;

	cap.open(file);

	//check if success
	if (!cap.isOpened()) {
		cout << "Error: video file could not be loaded. Aborting." << endl << endl;
		return -1;
	}

	//analysis method selection
	cout << endl << " Choose analysis method [0 = ROI (L-K), 1 = point (L-K), 2 = grid (FB)]: ";
	cin >> func;

	//properties for each program function
	if (func == 0) {
		//L-K optical flow
		funcInt = 2;
		linesz = 1;

		cout << endl
			 << " Select two points to define ROI with the mouse. A third mouse click will reset the selection." << endl
			 << " Press ENTER when the selection is made." << endl;

	}
	else if (func == 1) {
		//point tracking
		funcInt = 3;
		linesz = 2;

		cout << endl
			 << " Select one point of interest with the mouse. A second mouse click will reset the selection." << endl
			 << " Press ENTER when the selection is made." << endl;
	}
	else if (func == 2) {
		//Farneback optical flow
		linesz = 1;

		cout << endl
			 << " Choose flow density [0 = sparse, 1 = dense]: ";
		cin >> flowDen;

		if (flowDen == 1) FBstep = 2;	//uses dense flow
		else if (flowDen == 0) FBstep = STEP;
		else {
			cout << "Error: invalid method choosen. Aborting." << endl << endl;
			return -1;
		}
	}
	else {
		cout << "Error: invalid method choosen. Aborting." << endl << endl;
		return -1;
	}

	for (;;) {
		//initial behavior
		if (cnt == 0) {
			cap >> frame;		//gets new frame
			if(!frame.data) {
				cout << "Error: no video data found." << endl << endl;
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
				imshow("ROI Selection", roiFrame);
				setMouseCallback("ROI Selection", roiSelection);
				waitKey(0);

				//if points have been selected and ROI defined
				if (roiPts.size() == funcInt) {
					//creates ROI and ROI mask
					roi = frame(roiBox);
					cvtColor(roi, nextFrame, CV_BGR2GRAY);

					if (func == 1) {
						//prepares vectors for point tracking
						nextPts.push_back(roiPts[2]);
						oriVec.push_back(roiPts[2]);
					}
				}
				else {
					cout << "Error: not enough points selected to form ROI." << endl << endl;
					return -1;
				}
			}
			else {
				prevFrame = nextFrame.clone();

				//overrides counter
				cnt = 1;
			}

			//closes ROI selection window
			destroyWindow("ROI Selection");
		}

		if (func == 0) {
			//gets image features
			goodFeaturesToTrack(nextFrame, nextPts, nPts, qLevel, minDist);
		}

		//initializes prevPts vector
		if (func == 0 || func == 1) prevPts = nextPts;

		prevFrame = nextFrame.clone();		//first frame is the same as last one
		cap >> frame;						//gets a new frame from camera
			
		//ends tracking if video ends
		if(!frame.data) {
			cout << endl << " Video has ended. Tracking Stopped." << endl << endl;
			break;
		}

		//resizes and converts to grayscale
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
							//defines lines' points
							x1 = roiPts[0].x + prevPts[i].x;	//initial pts
							y1 = roiPts[0].y + prevPts[i].y;
							x2 = x1 + oriVec[i].x*LK_SCALE;		//final pts = initial pts +
							y2 = y1 + oriVec[i].y*LK_SCALE;		// + orientation vcs (w/ scale)

							//vector color attribution
							cidx = PointLoc(oriVec[i]);
		
							line(frame, Point(x1, y1), Point(x2, y2), color[cidx], linesz);
						}
					}
				}
				
				//draws compass
				drawCompass(frame);
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
			for (int i = 0; i < frame.rows; i += FBstep) {
				for (int j = 0; j < frame.cols; j += FBstep) {
					flowPt = flow.at<Point2f>(i, j)*FB_SCALE;

					if (traj) {
						if ((flowPt.x > FLOW_SZ || flowPt.x < -FLOW_SZ) && (flowPt.y > FLOW_SZ || flowPt.y < -FLOW_SZ)) {
							//stores flow in orientations vector
							oriVec.push_back(flowPt);

							//vector color attribution
							cidx = PointLoc(flowPt);

							line(frame, Point(j, i), Point(j + flowPt.x, i + flowPt.y), color[cidx], linesz);
						}
						else continue;
					}
				}
			}

			//draws compass
			drawCompass(frame);
		}

		imshow("Movement Tracking", frame);	//shows original capture

		//pauses and exits videos
		char key = waitKey(33);

		//if "space" pressed, pauses. If "esc" pressed, exits program
	    if (key == 32) {
	    	key = 0;

	    	//calculates orientation's histogram and displays it
	    	if (func == 0 || func == 2) {
	    		Mat histImg;

	    		getHist(histImg, oriVec);

	    		imshow("Orientation's Histogram", histImg);
	    	}

	    	while (key != 32) {
	    		imshow("Movement Tracking", frame);
	    		key = waitKey(5);
	    		if (key == 27) break;
	    	}
	    	if (key == 27) {
	    		cout << endl << " Tracking aborted by user. Exiting." << endl << endl;
	    		break;
	    	}
	    	else key = 0;

	    	//closes histogram window
	    	destroyWindow("Orientation's Histogram");
	    }
	    //if "S" pressed, toggles trajectories
	    if (key == 's' || key == 'S') traj = !traj;
	    //if "esc" pressed, exits
	    if (key == 27) {
	    	cout << endl << " Tracking aborted by user. Exiting." << endl << endl;
	    	break;
	    }
	}

	return 0;
}


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
						rectangle(roiFrame, roiPts[0], roiPts[1], Scalar(255, 255, 0), 2);
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

			imshow("ROI Selection", roiFrame);
	}
}


int PointLoc(Point2f pt) {
	// This function returns an integer value corresponding to the point location
	int a;

	if (pt.x > 0 && pt.y > 0) a = 0;
	else if (pt.x < 0 && pt.y > 0) a = 1;
	else if (pt.x < 0 && pt.y < 0) a = 2;
	else if (pt.x > 0 && pt.y < 0) a = 3;
	else a = 4;

	return a;
}


void drawCompass(Mat& frame) {
	// This function draws a compass at the top left corner of the image
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE+10*RES_SCALE, 20*RES_SCALE+10*RES_SCALE), color[0], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE-10*RES_SCALE, 20*RES_SCALE+10*RES_SCALE), color[1], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE-10*RES_SCALE, 20*RES_SCALE-10*RES_SCALE), color[2], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE+10*RES_SCALE, 20*RES_SCALE-10*RES_SCALE), color[3], 2);
}


void getHist(Mat& histImg, vector<Point2f> vec) {
	// This function returns the histogram of the orientation's frequencies
	int histW = 512, histH = 400;				//sets size of window
	Mat histImage(histH, histW, CV_8UC3, Scalar(0, 0, 0));

	histImg = histImage.clone();

	//orientation freq
	int freq[5] = {0, 0, 0, 0, 0};

	//calculates frequency of orientations
	for (unsigned int i = 0; i < vec.size(); i++) {
		freq[PointLoc(vec[i])]++;
	}
	
	//displays histogram elements
	int len = 43;
	line(histImg, Point(len, 0), Point(len, 400), color[4], 1);
	putText(histImg, "250", Point(10, 348), FONT_HERSHEY_PLAIN, 1, color[4]);
	line(histImg, Point(len/2, 350), Point(len, 350), color[4], 1);
	line(histImg, Point(len+1, 350), Point(histW, 350), Scalar(128, 128, 128), 1, 4);
	putText(histImg, "500", Point(10, 298), FONT_HERSHEY_PLAIN, 1, color[4]);
	line(histImg, Point(len/2, 300), Point(len, 300), color[4], 1);
	line(histImg, Point(len+1, 300), Point(histW, 300), Scalar(128, 128, 128), 1, 4);
	putText(histImg, "1000", Point(0, 198), FONT_HERSHEY_PLAIN, 1, color[4]);
	line(histImg, Point(len/2, 200), Point(len, 200), color[4], 1);
	line(histImg, Point(len+1, 200), Point(histW, 200), Scalar(128, 128, 128), 1, 4);
	putText(histImg, "1500", Point(0, 98), FONT_HERSHEY_PLAIN, 1, color[4]);
	line(histImg, Point(len/2, 100), Point(len, 100), color[4], 1);
	line(histImg, Point(len+1, 100), Point(histW, 100), Scalar(128, 128, 128), 1, 4);

	//displays histogram bars
	int binW = cvRound((double)(histW-44)/5);	//width of bins
	int maxVal = 2000;							//frequency max value (can actually be higher!)

	for (int i = 0; i < 5; i++) {
		int binsz = freq[i]*histH/maxVal;		//bin height

		//draws bin
		rectangle(histImg, Point(len+1+i*binW, histH), Point(len+(i+1)*binW, histH-binsz), color[i], CV_FILLED);
		
		//displays bin value
		stringstream ss;
		ss << freq[i];
		string str = ss.str();
		putText(histImg, str, Point((i+1)*binW-len, histH-binsz-2), FONT_HERSHEY_PLAIN, 1, color[i]);
	}
}