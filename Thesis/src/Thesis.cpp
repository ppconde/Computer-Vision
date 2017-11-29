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
#include <string>
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
#define HIST_INIT	1
#define TABLE_INIT 1
#define RES_SCALE	1.5		//image resizing scale
#define FLOW_SZ 	0.5		//minimum flow size
#define LK_SCALE	5		//Lucas-Kanade flow scale
#define PI 			3.14159	//pi
#define HIST_SCALE 10

//global variables
Mat roiFrame, roiAux, auxBlackFrame, frame;
Rect roiBox;
vector<Point2f> roiPts;		//points defining a ROI
vector<Point2f> actualPts;
vector<vector <Point2f> > pointsHistory; //Point history
vector<Vec4i> hierarchy;
vector <Point> ROI_Poly;    //Region of interest polygon
vector <Point2f> scaledRoiPts;
vector <float> travRes;    //Traveled Space values
vector <float> displacement;   //Displacement values
vector <Point2f> oriDistVec;
vector <float> vecDist(9, 0); //Euclidian distance for the histogram

int frameHeightTop, frameHeightBot, actualFrameHeight;
double travSpace = 0;   //Traveled space, ROI tracking method
float dispSpace = 0;   //Displacement, ROI tracking method
unsigned int cnt = 0;		//mouse clicks counter
bool cntFirst = true;
unsigned int func;		//for program function
int frameCnt = 0;     //frame counter
int ultraScale;

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
static void roiSelection(int, int, int, int, void*);

int VecOrientation(Point2f);

void drawCompass(Mat&);

Mat getHist(const vector<Point2f>&);

void getTable(Mat& tableImg, int rows, int cols, vector <float> displacement, vector <float> travRes);

void roi_polygon(vector<Point2f>, Mat);


int pnpoly(int nvert, vector<Point2f> roiPts, Point2f prevPt);

Point2f getMC(vector <Point2f> points);

//main routine
int main(int argc, char** argv)
{
  //C++

	int sel;							//video selection
	bool traj = TRAJ_INIT;				//trajectory visualization
	bool hist = HIST_INIT;				//histogram visualization
  bool table = TABLE_INIT;      //Table visualization
	char file[20];
	int linesz;							//line thickness
  float initShapeH = 0.0, initShapeW = 0.0, shapeH = 0.0, shapeW = 0.0; //Shape width and height information

	//matrixes declaration
	Mat prevFrame, nextFrame, roi, flow, auxFrame;
	UMat flowUMat;

	//vectors  and points declaration
  vector<Point> pointInt;       //Int Points for polygon
	vector<Point2f> oriVec;				//orientations vector
	vector<Point2f> prevPts, nextPts;	//Point to analyze
	vector<uchar> status;				//L-K status vector
	vector<float> err;					//L-K error vector

	//L-K optical flow configuration
	int nPts = 5000;					//number of features
	double qLevel = 0.01;				//quality level
	double minDist = 0.01;				//min euclidian distance

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
		 << " It is possible to pass videos as arguments of the program. If a video" << endl
		 << "is given, it will be analyzed." << endl
		 << " After the needed configuration, which will be prompted to the user, a" << endl
		 << "window with the video feed will be displayed." << endl
		 << " The following commands are available:" << endl
		 << "\tSpace\t- pauses the video" << endl
		 << "\tS\t- toggles movement trajectories (hidden by default)" << endl
		 << "\tH\t- toggles histogram (hidden by default)" << endl
     << "\tT\t- toggles movement's table (hidden by default)" << endl
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

  cout << endl << " Choose video scale in mm (e.g. 20, 30, etc): ";
	cin >> ultraScale;

	//analysis method selection
	cout << endl << " Choose analysis method:" <<endl;
  cout << "   0 = ROI Tracking"<< endl;
  cout << "   1 = Point-wise Tracking" << endl;
  cout << "   2 = Contour Tracking" << endl;
	cin >> func;

	//properties for each program function
	if (func == 0 || func == 2 ) {
		linesz = 1;

		cout << endl
			 << " Select at least two points to define a ROI with the mouse. A right mouse click will reset the selection." << endl
    			 << " Press ENTER when the selection is made." << endl;

	}
	else if (func == 1) {
		linesz = 1;

		cout << endl
			 << " Select one or more points of interest with the mouse. A right mouse click will reset the selection." << endl
			 << " Press ENTER when the selection is made." << endl;
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

			//converts frame to grayscale
			cvtColor(frame, nextFrame, CV_BGR2GRAY);

			//images for selection
			roiFrame = frame.clone();
      medianBlur(roiFrame, roiFrame, 5);

			roiAux = frame.clone();
      auxBlackFrame = Mat::zeros(frame.size(), CV_8UC1);

      Mat binaryFrame(nextFrame.size(), CV_8UC1);
      threshold(nextFrame, binaryFrame, 50, 255, THRESH_BINARY);

      //Measure actual video pixel height
       for(int i = 0; i < binaryFrame.rows ; i++){
         //cout << binaryFrame.at<uchar>(i, binaryFrame.cols-5) << endl;
         if((uchar)binaryFrame.at<uchar>(i, binaryFrame.cols-5) != 0){
           frameHeightTop = i;
           break;
         }
       }

       for(int i = binaryFrame.rows; i > 0 ; i--){
         //cout << binaryFrame.at<uchar>(i, binaryFrame.cols-5) << endl;
         if((uchar)binaryFrame.at<uchar>(i, binaryFrame.cols-5) != 0){
           frameHeightBot = i;
           break;
         }
       }

       actualFrameHeight = frameHeightBot - frameHeightTop;

			//mouse callback for selecting ROI
			imshow("ROI Selection", frame);
			setMouseCallback("ROI Selection", roiSelection);
      waitKey(0);

      if(func == 0){
        line(frame, roiPts[roiPts.size()-1], roiPts[0], color[4], 2, 8);
        imshow("ROI Selection", frame);
        waitKey(0);


        //Initializes prevPts vector
        prevPts = nextPts;

        //Region of interest rectangle
        roiBox = boundingRect(roiPts);
        nextFrame = nextFrame(roiBox);
      }
      else if(func == 1 || func  == 2){
        for(unsigned int k = 0; k < roiPts.size(); k++){
          pointsHistory.push_back(vector<Point2f>());
        }
      }
			else if (roiPts.size() >= 1){
				if (func == 0 || func == 1 || func == 2) {
          for(size_t i=2; i<=roiPts.size()-1; i=i+3){
            //prepares vectors for point tracking
            nextPts.push_back(roiPts[i]);
            oriVec.push_back(roiPts[i]);
          }
				}
			}
      else if (func == 2){
        line(frame, roiPts[roiPts.size()-1], roiPts[0], color[4], 1, 8);
        imshow("ROI Selection", frame);
        waitKey(0);
      }
			else {
				cout << "Error: not enough points selected to form ROI." << endl << endl;
				return -1;
			}
			//closes ROI selection window
			destroyWindow("ROI Selection");
		}
    if( func == 0) goodFeaturesToTrack(nextFrame, nextPts, nPts, qLevel, minDist);

    //initializes prevPts vector
    prevPts = nextPts;
    //stores last frame analysis
    auxFrame = frame.clone();

		prevFrame = nextFrame.clone();		//first frame is the same as last one
		cap >> frame;						//gets a new frame from camera

		//ends tracking if video ends
		if(!frame.data) {
			if (func == 0 || func == 1 || func == 2) {
				imshow("Movement Tracking", auxFrame);
				waitKey(0);
			}
			cout << endl << " Video has ended. Tracking Stopped." << endl << endl;
			break;
		}

    cvtColor(frame, nextFrame, CV_BGR2GRAY);

		if (func == 0 || func == 1 || func == 2)
    {
			//calculates optical flow using Lucas-Kanade
      if(func == 0)
      {

        //Stores original resized frame
        auxFrame = frame.clone();

        //Region of interest rectangle
        roiBox = boundingRect(roiPts);

        //Mat or image of the roiBox
        nextFrame = nextFrame(roiBox);

        //Calculates ROI Points coordinates related to the roiBox
        if(cntFirst){
            auxBlackFrame = auxBlackFrame(roiBox);
            scaledRoiPts = roiPts;
            for( unsigned int i = 0; i < scaledRoiPts.size(); i++){
              scaledRoiPts[i].x = scaledRoiPts[i].x - roiBox.x;
              scaledRoiPts[i].y = scaledRoiPts[i].y - roiBox.y;
            }
            cntFirst=false;
        }


        calcOpticalFlowPyrLK(prevFrame, nextFrame, prevPts, nextPts, status, err);

        //draws compass
        drawCompass(frame);

				//calculates orientations vector
				subtract(nextPts, prevPts, oriVec);

        //Draws ROI Polygon
        roi_polygon(roiPts, frame);

        //draws motion lines on display
        for (unsigned int i = 0; i < prevPts.size(); i++) {
          if (status[i]) {
            if(pnpoly(cnt, scaledRoiPts, prevPts[i])){
              x1 = roiBox.x + prevPts[i].x;	//initial pts
              y1 = roiBox.y + prevPts[i].y;
              x2 = x1 + oriVec[i].x*LK_SCALE;		//final pts = initial pts +
              y2 = y1 + oriVec[i].y*LK_SCALE;		// + orientation vcs (w/ scale)

               //vector color attribution
               cidx = VecOrientation(oriVec[i]);
               line(frame, Point(x1, y1), Point(x2, y2), color[cidx], linesz);
            }
          }
        }
			}
      //Multiple point tracking
      if(func == 1){
        drawCompass(frame);
        for(unsigned int k = 0; k < actualPts.size(); k++) {
            pointsHistory[k].push_back(actualPts[k]);
        }

        vector<Point2f> out;
        Point2d oriDispVec;
        vector <float> travPts(pointsHistory.size(),0);

        travRes.resize(pointsHistory.size());
        displacement.resize(pointsHistory.size());

        //Calculate Optical Flow Lucas-Kanade method
        calcOpticalFlowPyrLK(prevFrame, nextFrame, actualPts, out, status, err);
        actualPts = out;
        for(unsigned int i = 0; i < pointsHistory.size(); i++){
          vector < Point2f > points = pointsHistory[i];

          for(unsigned int k = 1; k < points.size(); k++){
            //Draws line, if it's the actual point then draws yellow line
            line(frame, points[k-1], points[k], (k == points.size() - 1) ? color[11] : color[VecOrientation(points[k]-points[k-1])], 2);
          }
          //Initializes all the orientation vectors with (0,0)
          if(frameCnt == 0){
            for(unsigned int j = 0; j < pointsHistory.size(); j++){
              oriDistVec.push_back(Point2f(0,0));
            }
            //Initializes all the travRes and TravPts with (pointsHistory.size(), 0)
            travRes.assign((int)pointsHistory.size(), 0);
            travPts.assign((int)pointsHistory.size(), 0);
          }
          if(frameCnt >0  ){
            //Calculates orientation for each point
              oriDistVec[i] = points[points.size()-1] - points[points.size()-2];
          }
          //Calculates distance in mm
          travPts[i] = (ultraScale*hypot((4/3)*oriDistVec[i].x, oriDistVec[i].y)/actualFrameHeight);

          //Calculates traveled space
          travRes[i] = travPts[i] + travRes[i];
          //add(travRes, travPts, travRes);

          //Calculates displacement
          oriDispVec = (points[points.size()-1]-points[0]);

          displacement[i] = ultraScale*hypot((4/3)*oriDispVec.x, oriDispVec.y)/actualFrameHeight;

          stringstream ss_i, ss_displacement, ss_travRes;
          ss_i << i;
          ss_displacement << trunc(displacement[i]*10)/10;
          ss_travRes << trunc(travRes[i]*10)/10;

          putText(frame, ss_i.str(), Point(points[0].x, points[0].y+20), FONT_HERSHEY_PLAIN, 1, color[8], 1);
        }
      }
      //Contour tracking
      else if(func == 2){
        drawCompass(frame);
        for(unsigned int k = 0; k < actualPts.size(); k++){
            pointsHistory[k].push_back(actualPts[k]);
        }

        vector<Point2f> out;

        calcOpticalFlowPyrLK(prevFrame, nextFrame, actualPts, out, status, err);

        for(unsigned int i=0; i<out.size(); i++){
          circle(frame, out[i], 5, color[0], 1);
        }

        //Traveled space and displacement orientation points
        Point2d oriDistVec, oriDispVec;

        //Calculate center of mass of ROI
        Point2f massCenter = getMC(actualPts);

        Point2f massCenterNew = getMC(out);

        //Calculate ROIpts center of mass
        Point2f massCenterInit = getMC(roiPts);

        circle(frame, Point2f(massCenterNew.x, -massCenterNew.y), 5, color[5], 2);
        oriDistVec = massCenterNew - massCenter;

        //Calculate traveled space
        travSpace += (ultraScale*hypot((4/3)*oriDistVec.x, oriDistVec.y)/actualFrameHeight);

        //Calculate displacement
        oriDispVec = massCenterNew - massCenterInit;
        dispSpace = (ultraScale*hypot((4/3)*oriDispVec.x, oriDispVec.y)/actualFrameHeight);

        stringstream ss_dispSpace, ss_travSpace;
        ss_dispSpace << trunc(dispSpace*10)/10;
        ss_travSpace << trunc(travSpace*10)/10;

        putText(frame, "Displacement: " + ss_dispSpace.str() + " mm", Point(20, 420), FONT_HERSHEY_PLAIN, 1, color[8]);
        putText(frame, "Traveled Space: " + ss_travSpace.str() + " mm", Point(300, 420), FONT_HERSHEY_PLAIN, 1, color[8]);

        if(frameCnt == 1){
          Rect initShape = boundingRect(roiPts);
          initShapeH = initShape.height;
          initShapeW = initShape.width;

          //Initial height and width of shape;
          initShapeH = initShapeH*ultraScale/actualFrameHeight;
          initShapeW = initShapeW*(4/3)*ultraScale/actualFrameHeight;
        }

        //Calculate shape selection height and width
        Rect shape = boundingRect(actualPts);
        shapeH = shape.height;
        shapeW = shape.width;
        shapeH = shapeH*ultraScale/actualFrameHeight;
        shapeW = shapeW*(4/3)*ultraScale/actualFrameHeight;

        stringstream ss_initShapeH, ss_initShapeW, ss_shapeH, ss_shapeW;
        ss_initShapeH << trunc(initShapeH*10)/10;
        ss_initShapeW << trunc(initShapeW*10)/10;
        ss_shapeH << trunc(shapeH*10)/10;
        ss_shapeW << trunc(shapeW*10)/10;

        putText(frame, "Init height: " + ss_initShapeH.str() + " mm", Point(20, 440), FONT_HERSHEY_PLAIN, 1, color[8]);
        putText(frame, "Init width: " + ss_initShapeW.str() + " mm", Point(20, 460), FONT_HERSHEY_PLAIN, 1, color[8]);
        putText(frame, "Current height: " + ss_shapeH.str() + " mm", Point(300, 440), FONT_HERSHEY_PLAIN, 1, color[8]);
        putText(frame, "Current width: " + ss_shapeW.str() + " mm", Point(300, 460), FONT_HERSHEY_PLAIN, 1, color[8]);
        actualPts = out;

        //Draw polygon using points
        roi_polygon(out, frame);
        oriVec.clear();
        oriVec.push_back(oriDistVec);
      }
      frameCnt++;
    }

    //calculates orientation's histogram and displays it
		if (table && func == 1) {
			Mat tableImg;
      getTable(tableImg, pointsHistory.size(), 3, displacement, travRes);
      imshow("Movement's Table", tableImg);
		}
		else destroyWindow("Movement's Table");

		//calculates orientation's histogram and displays it
		if (hist && (func == 0 || func == 2)) {
			Mat histImg = getHist(oriVec);
	    imshow("Orientation's Histogram", histImg);
		}
		else destroyWindow("Orientation's Histogram");

		if (traj) imshow("Movement Tracking", frame);	//shows frame with flow
		else {imshow("Movement Tracking", auxFrame);		//shows original capture
  }

		//user input behaviour
		char key = waitKey(33);

		//if "space" pressed, pauses
	    if (key == 32) {
	    	key = 0;

	    	while (key != 32) {
	    		key = waitKey(5);

	    		//displays frame
	    		if (traj) imshow("Movement Tracking", frame);
				else imshow("Movement Tracking", auxFrame);

	    		//checks for trajectory toggle
				if (key == 's' || key == 'S') traj = !traj;

	    		//checks for histogram toggle
	    		if (key == 'h' || key == 'H') {
	    			if (hist && func != 1) {
	    				hist = 0;
						destroyWindow("Orientation's Histogram");
					}
					else {
						hist = 1;
						Mat histImg = getHist(oriVec);
				    imshow("Orientation's Histogram", histImg);
					}
				}

	    		if (key == 27) break;
	    	}
	    }

	    //if "S" pressed, toggles trajectories
	    if (key == 's' || key == 'S') traj = !traj;

	    //if "H" pressed, toggles histogram
	    if (key == 'h' || key == 'H') hist = !hist;

      if (key == 't' || key == 'T') table = !table;

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
				if (func == 0) {
					//point selection
					Point selected = Point(x, y);
					roiPts.push_back(selected);

					//displays selected point
					circle(frame, selected, 5, color[0], 1);
          //circle(auxBlackFrame, selected, 5, color[0], 1);
          if(cnt>=2){
            line(frame, roiPts[cnt-2], roiPts[cnt-1], color[4], 2, 8);
            line(auxBlackFrame, roiPts[cnt-2], roiPts[cnt-1], color[4], 1, 8);
          }
				}
        if (func == 1) {
					cnt++;
					//point selection
					Point selected = Point(x, y);
					roiPts.push_back(selected);

          actualPts.push_back(Point(x, y));

					//displays selected point
					circle(frame, selected, 5, color[0], 1);

					//stores ROI
					roiBox = boundingRect(actualPts);
				}
        else if(func == 2)
        {
          Point selected = Point(x,y);
          roiPts.push_back(selected);
          actualPts.push_back(selected);

          //displays selected point
          circle(frame, selected, 5, color[0], 1);
          if(cnt>=2)
          {
            line(frame, roiPts[cnt-2], roiPts[cnt-1], color[4], 2, 8);
          }
        }
        imshow("ROI Selection", frame);
      break;

    case CV_EVENT_RBUTTONDOWN:
      //flushes point vector
      frame = roiAux.clone();
      roiPts.clear();
      actualPts.clear();
      cnt = 0;
      imshow("ROI Selection", frame);
      break;
	}
}

int VecOrientation(Point2f pt1) {
	// This function returns an int value corresponding to the vector orientation
	int a;

  double ang = atan2(pt1.y, pt1.x) * (180 / PI);

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


Mat getHist(const vector<Point2f>& vec) {
	// This function returns the histogram of the orientation's frequencies
	int histW = 512, histH = 600;				//sets size of window
	Mat histImg(histH, histW, CV_8UC3, Scalar(0, 0, 0));

	//orientation freq
	const int nbins = 9;
	static float freq[nbins] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

	for (unsigned int i = 0; i < vec.size(); i++) {
    int ori = VecOrientation(vec[i]);
    //Calculates euclidian distance of orientation vector
    vecDist[ori] = ultraScale * hypot((4/3)*vec[i].x, vec[i].y) / (actualFrameHeight*vec.size());

    //calculates frequency of orientations
		freq[ori] += vecDist[ori];
	}

	//displays initial histogram elements
	int len = 43;
	line(histImg, Point(len, 0), Point(len, histH), color[8], 1);
	putText(histImg, "7,5", Point(15, histH*1/4-5), FONT_HERSHEY_PLAIN, 1, color[8]);
  line(histImg, Point(len/2, histH*1/4), Point(len, histH*1/4), color[8], 1);
	line(histImg, Point(len+1, histH*1/4), Point(histW, histH*1/4), color[10], 1, 4);
	putText(histImg, "5", Point(20, histH*2/4-5), FONT_HERSHEY_PLAIN, 1, color[8]);
  line(histImg, Point(len/2, histH*2/4), Point(len, histH*2/4), color[8], 1);
  line(histImg, Point(len+1, histH*2/4), Point(histW, histH*2/4), color[10], 1, 4);
	putText(histImg, "2,5", Point(15, histH*3/4-5), FONT_HERSHEY_PLAIN, 1, color[8]);
  line(histImg, Point(len/2, histH*3/4), Point(len, histH*3/4), color[8], 1);
  line(histImg, Point(len+1, histH*3/4), Point(histW, histH*3/4), color[10], 1, 4);
	putText(histImg, "1,25", Point(5, histH*7/8-5), FONT_HERSHEY_PLAIN, 1, color[8]);
  line(histImg, Point(len/2, histH*7/8), Point(len, histH*7/8), color[8], 1); //LEFT LINE
  line(histImg, Point(len+1, histH*7/8), Point(histW, histH*7/8), color[10], 1, 4);  //RIGHT LINE
  putText(histImg, "mm", Point(10, 20), FONT_HERSHEY_PLAIN, 1, color[8]);

	//displays histogram bars
	int binW = cvRound((double)(histW-44)/nbins);	//width of bins
	int const maxVal = 10;								//frequency max value (can actually be higher!)
  float const res = histH/maxVal;
	for (int i = 0; i < nbins; i++) {
		float binsz = freq[i] *	res;		//bin height

		//draws bin
		rectangle(histImg, Point(len + 1.0 + i*binW, histH), Point(len+(i+1.0)*binW, histH-binsz), color[i], CV_FILLED);

		//displays bin value
		stringstream ss;
		ss << trunc(freq[i]*10)/10;
		string str = ss.str();

		if (binsz <= histH - 15) {
			putText(histImg, str, Point((i+1)*binW, histH-binsz-2), FONT_HERSHEY_PLAIN, 1, color[i]);
		}
		else {
			putText(histImg, str, Point((i+1)*binW, 20), FONT_HERSHEY_PLAIN, 1, color[9]);
		}
	}

  return histImg;
}

void roi_polygon(vector<Point2f> roiPts, Mat frame)
{
  //draws ROI polygon
  int i=0;
  do{
    line(frame, roiPts[i], roiPts[i+1], color[4], 1, 8);
    i++;
  }while(i<static_cast<int>(roiPts.size())-1);
  if(i==static_cast<int>(roiPts.size())-1)
  {
    line(frame, roiPts[static_cast<int>(roiPts.size())-1], roiPts[0], color[4], 1, 8);
  }
}

int pnpoly(int nvert, vector<Point2f> roiPts, Point2f prevPt)
{
  /*
  nvert: Number of vertices in the polygon.
  vertx, verty: Arrays containing the x- and y-coordinates of the polygon's vertices.
  prevPt.x, prevPt.y: X- and y-coordinate of the test point.
  */
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((roiPts[i].y>prevPt.y) != (roiPts[j].y>prevPt.y)) &&
     (prevPt.x < (roiPts[j].x-roiPts[i].x) * (prevPt.y-roiPts[i].y) / (roiPts[j].y-roiPts[i].y) + roiPts[i].x) )
       c = !c;
  }
  return c;
}
void getTable(Mat& tableImg, int rows, int cols, vector <float> displacement, vector <float> travRes){
  int tableW = 512, tableH = 600;				//sets size of window
  Mat tableImage(tableH, tableW, CV_8UC3, Scalar(0, 0, 0));
  tableImg = tableImage.clone();
  rows = rows+1;
  int widthStep = 30, heightStep = (tableH/rows)*1/2;


  //Draws table
  for(int i = 1; i < rows; i ++){
    for(int j = 1; j < cols; j++){
      line(tableImg, Point((tableW/cols)*j, 0), Point((tableW/cols)*j, tableH),Scalar(255, 255, 255));

      stringstream ss_i, ss_displacement, ss_travRes;
      ss_i << i-1;
      ss_displacement << trunc(displacement[i-1]*10)/10;
      ss_travRes << trunc(travRes[i-1]*10)/10;

      //Index of point
      putText(tableImg, ss_i.str(), Point(((tableW/cols)/2), heightStep + (tableH/rows)*i), FONT_HERSHEY_PLAIN, 1, color[8]);

      //Displacement values
      putText(tableImg, "Displacement", Point((tableW/cols)+widthStep, heightStep), FONT_HERSHEY_PLAIN, 1, color[8]);
      putText(tableImg, "(mm)", Point((tableW/cols)+widthStep, heightStep + 20), FONT_HERSHEY_PLAIN, 1, color[8]);
      putText(tableImg, ss_displacement.str(), Point((tableW/cols)+widthStep, heightStep + (tableH/rows)*i), FONT_HERSHEY_PLAIN, 1, color[8]);

      //Traveled space values
      putText(tableImg, "Trav Space", Point((tableW/cols)*(cols-1) +widthStep, heightStep), FONT_HERSHEY_PLAIN, 1, color[8]);
      putText(tableImg, "(mm)", Point((tableW/cols)*(cols-1) +widthStep, heightStep + 20), FONT_HERSHEY_PLAIN, 1, color[8]);
      putText(tableImg, ss_travRes.str(), Point((tableW/cols)*(cols-1) +widthStep, heightStep + (tableH/rows)*i), FONT_HERSHEY_PLAIN, 1, color[8]);

    }
    line(tableImg, Point(0, (tableH/rows)*i), Point(tableW, (tableH/rows)*i),Scalar(255, 255, 255));
  }
}
Point2f getMC(vector <Point2f> points){
  double totalX=0.0, totalY=0.0;
  for(unsigned int i = 0; i < points.size(); i++){
    totalX+=points[i].x;
    totalY+=points[i].y;
  }
  Point2f massCenter(totalX/points.size(), -totalY/points.size()); // condition: size != 0
  return massCenter;
}
