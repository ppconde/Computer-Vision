/*===========================================*/
/*											 */
/*		MUSCLE MOVEMENT QUANTIFICATION 		 */
/*			COMPUTER VISION PROJECT 		 */
/*											 */
/*	Authors: Pedro Conde			 */
/*											 */
/*	Date:	 March 2017					 */
/*											 */
/*	Libraries used: OpenCV 3.1.0			 */
/*											 */
/*===========================================*/


#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

//namespace declaration
using namespace std;
using namespace cv;

//pre-processor defines
#define TRAJ_INIT	0
#define HIST_INIT	0
#define RES_SCALE	1.5		//image resizing scale
#define STEP 		10		//Farneback grid spacing in pix
#define FLOW_SZ 	0.5		//minimum flow size
#define LK_SCALE	5		//Lucas-Kanade flow scale
#define FB_SCALE	10		//Farneback flow scale
#define PI 			3.14159	//pi

//global variables
Mat roiFrame, roiAux;
Rect roiBox;
vector<Point2f> roiPts;		//points defining a ROI
vector<Point2f> actualPts;
vector<vector <Point2f> > pointsHistory; //Point history
vector <vector <Point> > contours_poly;

int func;							//program function type
int linesz;							//line thickness
unsigned int cnt = 0;		//mouse clicks counter
unsigned int funcInt;		//for program function
int sel;							//video selection
bool traj = TRAJ_INIT;				//trajectory visualization
bool hist = HIST_INIT;				//histogram visualization
char file[20];
char auxch[50];
bool drmed = 0;

//matrixes declaration
Mat frame, prevFrame, nextFrame, roi, flow, auxFrame;
UMat flowUMat;

//vectors  and points declaration
vector<Point> pointInt;       //Int Points for polygon
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

//float x1, y1, x2, y2;				//point coords for drawing

//color structure for segmentation
Scalar color[] =
{
    Scalar(0, 0, 255),				//red
    Scalar(0, 128, 255),			//orange
    Scalar(0, 255, 128), 			//light green
	  Scalar(128, 255, 0),			//turquoise
	  Scalar(255, 255, 0),			//cyan
	  Scalar(255, 128, 0),			//"blue"
	  Scalar(255, 0, 128),			//purple
	  Scalar(128, 0, 255),			//pink
    Scalar(255, 255, 255),		//white
    Scalar(0, 0, 0),				  //black
    Scalar(128, 128, 128),			//gray
    Scalar(0, 255, 255)       //Yellow
};
int cidx;							//color index


//functions' declaration
int roiLK(void);
static void roiSelection(int, int, int, int, void*);

int VecOrientation(Point2f);

void drawCompass(Mat&);

void getHist(Mat&, vector<Point2f>);


//main routine
int main(int argc, char** argv) {

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
		 << " It is possible to pass videos as arguments of the program. If a video" << endl
		 << "is given, it will be analyzed." << endl
		 << " After the needed configuration, which will be prompted to the user, a" << endl
		 << "window with the video feed will be displayed." << endl
		 << " The following commands are available:" << endl
		 << "\tSpace\t- pauses the video" << endl
		 << "\tS\t- toggles movement trajectories (hidden by default)" << endl
		 << "\tH\t- toggles histogram (hidden by default)" << endl
		 << "\tEsc\t- exits the program" << endl
		 << " The movement quantification is color coded:" << endl
		 << "\tRed\t- E (-22.5º to 22.5º)\tOrange\t- NE (22.5º to 67.5º)" << endl
		 << "\tL Green\t- N (67.5º to 112.5º)\tM Green\t- NW (112.5º to 157.5º)" << endl
		 << "\tCyan\t- W (157.5º to 202.5º)\tBlue\t- SW (202.5º to 247.5º)" << endl
		 << "\tPurple\t- S (247.5º to 292.5º)\tPink\t- SE (292.5º to 337.5º)" << endl
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

  for(;;) //initial behaviour
  {
    if(cnt == 0)
    {
      cap >> frame;		//gets new frame
      if(!frame.data) {
        cout << "Error: no video data found." << endl << endl;
        break;
      }
      //resizes video frames for viewing purposes
      resize(frame, frame, Size(), RES_SCALE, RES_SCALE, INTER_CUBIC);

      //converts frame to grayscale
      cvtColor(frame, nextFrame, CV_BGR2GRAY);
    }

    //Method selection
    switch (func)
    { //ROI - Lukas-Kanade
      case 0:
        imshow("ROI Selection", roiFrame);
        roiLK();
      break;
      /*
      case 1:
      break;

      case 2:
      break;
      */
      default:
      cout << "Error: invalid method chosen. Aborting." << endl << endl;
      return -1;
    }
  }

}

int roiLK(void)
{
  //L-K optical flow
  funcInt = 2;
  linesz = 1;

  cout << endl
    <<  "Click with the LEFT button of the mouse to define the ROI. A RIGHT click will reset the selection" << endl
    <<  "Press ENTER when the selection is made." <<  endl;

  //images for selection
  roiFrame = frame.clone();
  roiAux = roiFrame.clone();


  //mouse callback for selecting ROI
  imshow("ROI Selection", roiFrame);
  setMouseCallback("ROI Selection", roiSelection);
  waitKey(0);
  line(roiFrame, roiPts[roiPts.size()-1], roiPts[0], color[4], 1, 8);
  imshow("ROI Selection", roiFrame);
  waitKey(0);

  Point pol_point[1][static_cast<int>(roiPts.size())];
  for(unsigned int k = 0; k < roiPts.size(); k++)
  {
      pointsHistory.push_back(vector<Point2f>());
      pol_point[0][k] = roiPts[k];
  }
  const Point* ppt[1] = { pol_point[0] };
  int npt[] = {static_cast<int>(roiPts.size())};

  fillPoly(roiFrame, ppt, npt, 1, color[2], 8);
  /*vector <Point> ROI_Poly;
  approxPolyDP(roiPts, ROI_Poly, 1.0, true);*/

  //Fit bounding rectangle
  //roiBox = boundingRect(ppt[1]);

  //if points have been selected and ROI defined
  if(roiPts.size() >=3)
  {
    //create ROI and ROI Mask
    roi = frame(roiBox);  //ROI Mask created
    cvtColor(roi, nextFrame, CV_BGR2GRAY);
  }
  else
  {
    cout << "Error: not enough points selected to form ROI" << endl << endl;
    return -1;
  }

  //closes ROI selection window
  destroyWindow("ROI Selection");

  return 0;
}

static void roiSelection(int event, int x, int y, int, void*) {
	// This function waits for user to select points that define a ROI or a point of interest
	switch (event) {
		case CV_EVENT_LBUTTONDOWN:
			cnt++;
			//point selection and ROI definition
				if (funcInt == 2) {
					//point selection
					Point selected = Point(x, y);
					roiPts.push_back(selected);

					//displays selected point
					circle(roiFrame, selected, 5, color[0], 1);
          if(cnt>=2)
          {
            line(roiFrame, roiPts[cnt-2], roiPts[cnt-1], color[4], 2, 8);
          }
          imshow("ROI Selection", roiFrame);
				}
				else if (funcInt == 3) {
					cnt++;

					//point selection
					Point selected = Point(x, y);
					roiPts.push_back(selected);

          actualPts.push_back(Point(x, y));

					//displays selected point
					circle(roiFrame, selected, 5, color[0], 1);

					//stores ROI
					roiBox = Rect(Point(1, 1), Point(roiFrame.size()));
				}
        imshow("ROI Selection", roiFrame);
      break;

    case CV_EVENT_RBUTTONDOWN:
      //flushes point vector
      roiFrame = roiAux.clone();
      roiPts.clear();
      cnt = 0;
      imshow("ROI Selection", roiFrame);
      break;
	}
}


int VecOrientation(Point2f pt1) {
	// This function returns an int value corresponding to the vector orientation
	int a;

	Point2f pt2 = Point(1, 0);		//pt for horizontal vector
	double l1, l2, dot, ang;		//dot product and lengths

	//vector's angle calculation
	l1 = sqrt(pt1.x*pt1.x + pt1.y*pt1.y);	//length of vector
	l2 = pt2.x;								//length of aux vector
	dot = pt1.x*pt2.x + pt1.y*pt2.y;		//dot product

	float dir = (pt1.cross(pt2) >= 0 ? 1.0 : -1.0);	//direction calculation (+ or -)

	ang = dir*acos(dot/(l1*l2))*180/PI;

	//orientation attribution
	if (ang >= -22.5 && ang < 22.5) a = 0;
	else if (ang >= 22.5 && ang < 67.5 ) a = 1;
	else if (ang >= 67.5 && ang < 112.5) a = 2;
	else if (ang >= 112.5 && ang < 157.5) a = 3;
	else if (ang >= 157.5 && ang <= 180) a = 4;
	else if (ang >= -180 && ang < -157.5) a = 4;
	else if (ang >= -157.5 && ang < -112.5) a = 5;
	else if (ang >= -112.5 && ang < -67.5) a = 6;
	else if (ang >= -67.5 && ang < -22.5) a = 7;
	else a = 8;

	return a;
}


void drawCompass(Mat& frame) {
	// This function draws a compass at the top left corner of the image
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE+12*RES_SCALE, 20*RES_SCALE), color[0], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE+10*RES_SCALE, 20*RES_SCALE+10*RES_SCALE), color[7], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE, 20*RES_SCALE+12*RES_SCALE), color[6], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE-10*RES_SCALE, 20*RES_SCALE+10*RES_SCALE), color[5], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE-12*RES_SCALE, 20*RES_SCALE), color[4], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE-10*RES_SCALE, 20*RES_SCALE-10*RES_SCALE), color[3], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE, 20*RES_SCALE-12*RES_SCALE), color[2], 2);
	arrowedLine(frame, Point(20*RES_SCALE, 20*RES_SCALE), Point(20*RES_SCALE+10*RES_SCALE, 20*RES_SCALE-10*RES_SCALE), color[1], 2);
}


void getHist(Mat& histImg, vector<Point2f> vec) {
	// This function returns the histogram of the orientation's frequencies
	int histW = 512, histH = 400;				//sets size of window
	Mat histImage(histH, histW, CV_8UC3, Scalar(0, 0, 0));

	histImg = histImage.clone();

	//orientation freq
	int nbins = 9;
	int freq[nbins] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

	//calculates frequency of orientations
	for (unsigned int i = 0; i < vec.size(); i++) {
		freq[VecOrientation(vec[i])]++;
	}

	//displays initial histogram elements
	int len = 43;
	line(histImg, Point(len, 0), Point(len, 400), color[8], 1);
	putText(histImg, "250", Point(10, 348), FONT_HERSHEY_PLAIN, 1, color[8]);
	line(histImg, Point(len/2, 350), Point(len, 350), color[8], 1);
	line(histImg, Point(len+1, 350), Point(histW, 350), color[10], 1, 4);
	putText(histImg, "500", Point(10, 298), FONT_HERSHEY_PLAIN, 1, color[8]);
	line(histImg, Point(len/2, 300), Point(len, 300), color[8], 1);
	line(histImg, Point(len+1, 300), Point(histW, 300), color[10], 1, 4);
	putText(histImg, "1000", Point(0, 198), FONT_HERSHEY_PLAIN, 1, color[8]);
	line(histImg, Point(len/2, 200), Point(len, 200), color[8], 1);
	line(histImg, Point(len+1, 200), Point(histW, 200), color[10], 1, 4);
	putText(histImg, "1500", Point(0, 98), FONT_HERSHEY_PLAIN, 1, color[8]);
	line(histImg, Point(len/2, 100), Point(len, 100), color[8], 1);
	line(histImg, Point(len+1, 100), Point(histW, 100), color[10], 1, 4);

	//displays histogram bars
	int binW = cvRound((double)(histW-44)/nbins);	//width of bins
	int maxVal = 2000;								//frequency max value (can actually be higher!)

	for (int i = 0; i < nbins; i++) {
		int binsz = freq[i]*histH/maxVal;			//bin height

		//draws bin
		rectangle(histImg, Point(len+1+i*binW, histH), Point(len+(i+1)*binW, histH-binsz), color[i], CV_FILLED);

		//displays bin value
		stringstream ss;
		ss << freq[i];
		string str = ss.str();

		if (binsz <= histH - 15) {
			putText(histImg, str, Point((i+1)*binW, histH-binsz-2), FONT_HERSHEY_PLAIN, 1, color[i]);
		}
		else {
			putText(histImg, str, Point((i+1)*binW, 20), FONT_HERSHEY_PLAIN, 1, color[9]);
		}

	}
}
